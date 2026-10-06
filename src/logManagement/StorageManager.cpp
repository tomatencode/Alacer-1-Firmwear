#include "StorageManager.hpp"

#include <algorithm>
#include <cstring>

namespace {

// Directory lives in the first two sectors (copies A/B for atomic swap).
// Data arena starts at 2*sectorSize and grows contiguously; files are never
// moved (delete = tombstone, space reclaimed by deleteAllFiles on ground).

constexpr uint32_t kDirMagic = 0x44474F4Cu; // 'LOGD' little-endian
constexpr uint16_t kDirVersion = 2;
constexpr uint8_t kCommitDone = 0x00;

constexpr size_t kHdrSize = 32;
constexpr size_t kEntrySize = 64;
constexpr size_t kNameSize = 32;

constexpr uint8_t kStateOpen = 0x7F;
constexpr uint8_t kStateClosed = 0x3F;
constexpr uint8_t kStateDeleted = 0x1F;

constexpr uint8_t kFlagNormal = 0xFF;
constexpr uint8_t kFlagRecovered = 0xFE;

constexpr uint32_t kLenOpen = 0xFFFFFFFF;

constexpr uint16_t kFrameMagic = 0xC0DE;
constexpr size_t kFrameHdrSize = 8;

// Entry layout (64B, little-endian):
//   0..31  name bytes (raw etl::string bytes, 0xFF-padded, includes NUL)
//   32     state (FF free / 7F open / 3F closed / 1F deleted, 1->0 only)
//   33     flags (FF normal / FE recovered)
//   34..37  start address
//   38..41  len (FFFFFFFF while open)
//   42     crcOpen  (name+OPEN+start, programmed at startFile)
//   43     crcClose (name+CLOSED+flags+start+len, programmed at close)
//   44..63  0xFF reserved
// Name bytes never change after startFile, so the in-place close only
// clears bits.

uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07)
                               : static_cast<uint8_t>(crc << 1);
        }
    }
    return crc;
}

void putU16(uint8_t *p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v);
    p[1] = static_cast<uint8_t>(v >> 8);
}

void putU32(uint8_t *p, uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        p[i] = static_cast<uint8_t>(v >> (8 * i));
    }
}

uint16_t getU16(const uint8_t *p) {
    return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

uint32_t getU32(const uint8_t *p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

bool isErased(const uint8_t *p, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        if (p[i] != 0xFF) {
            return false;
        }
    }
    return true;
}

uint16_t crc16Update(uint16_t crc, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

uint16_t crc16(const uint8_t *data, size_t len) {
    return crc16Update(0xFFFF, data, len);
}

uint16_t frameCrcRam(const uint8_t fhZeroed[8], const uint8_t *payload,
                     size_t payLen) {
    // fhZeroed must have crc bytes (6,7) already zeroed.
    uint16_t crc = crc16(fhZeroed, 8);
    return crc16Update(crc, payload, payLen);
}

uint8_t entryCrcOpen(const uint8_t name[kNameSize], uint32_t start) {
    uint8_t b[kNameSize + 5];
    memcpy(b, name, kNameSize);
    b[kNameSize] = kStateOpen;
    putU32(b + kNameSize + 1, start);
    return crc8(b, sizeof(b));
}

uint8_t entryCrcClose(const uint8_t name[kNameSize], uint8_t flags,
                      uint32_t start, uint32_t len) {
    uint8_t b[kNameSize + 10];
    memcpy(b, name, kNameSize);
    b[kNameSize] = kStateClosed;
    b[kNameSize + 1] = flags;
    putU32(b + kNameSize + 2, start);
    putU32(b + kNameSize + 6, len);
    return crc8(b, sizeof(b));
}

void nameToBytes(const char *name, size_t len, uint8_t out[kNameSize]) {
    memset(out, 0xFF, kNameSize);
    if (len >= kNameSize) {
        // No room for NUL: store all 32 bytes, parseEntry() treats a full
        // 32-byte field without NUL as a 32-char name.
        memcpy(out, name, kNameSize);
        return;
    }
    memcpy(out, name, len);
    out[len] = 0x00; // NUL-terminate so parseEntry() finds the length
}

void buildEntryImage(uint8_t img[64], const uint8_t name[kNameSize],
                     uint8_t state, uint8_t flags, uint32_t start,
                     uint32_t len) {
    memset(img, 0xFF, kEntrySize);
    memcpy(img + 0, name, kNameSize);
    img[32] = state;
    img[33] = flags;
    putU32(img + 34, start);
    putU32(img + 38, len);
    img[42] = entryCrcOpen(name, start);
    img[43] = (len == kLenOpen) ? 0xFF : entryCrcClose(name, flags, start, len);
}

void buildHeaderImage(uint8_t img[32], uint16_t seq, uint16_t activeCnt,
                      uint8_t commit) {
    memset(img, 0xFF, kHdrSize);
    putU32(img + 0, kDirMagic);
    putU16(img + 4, kDirVersion);
    putU16(img + 6, seq);
    putU16(img + 8, activeCnt);
    putU16(img + 10, 0xFFFF); // reserved
    img[12] = commit;
    img[13] = crc8(img, 12);
}

} // namespace

StorageManager::StorageManager(hardware::FlashChip &flashChip)
    : _flashChip(flashChip) {}

uint32_t StorageManager::activeDirAddr() const {
    return _activeDir == 0 ? 0 : _sectorSize;
}

uint32_t StorageManager::inactiveDirAddr() const {
    return _activeDir == 0 ? _sectorSize : 0;
}

uint32_t StorageManager::alignUp(uint32_t v, uint32_t align) {
    return (v + align - 1) / align * align;
}

uint32_t StorageManager::entryAddr(int slot) const {
    return activeDirAddr() + static_cast<uint32_t>(kHdrSize) +
           static_cast<uint32_t>(slot) * static_cast<uint32_t>(kEntrySize);
}

// === PART1 END ===
bool StorageManager::readHeader(uint32_t dirAddr, uint8_t img[32]) {
    if (!_flashChip.read(dirAddr, std::span(img, kHdrSize))) {
        return false;
    }
    if (getU32(img) != kDirMagic || getU16(img + 4) != kDirVersion) {
        return false;
    }
    if (img[12] != kCommitDone || crc8(img, 12) != img[13]) {
        return false; // torn compaction or never formatted
    }
    return true;
}

bool StorageManager::formatDir(uint32_t dirAddr, const FileInfo *files,
                               size_t fileCount, uint16_t seq) {
    if (!_flashChip.eraseSector(dirAddr)) {
        return false;
    }
    uint8_t img[64];
    for (size_t i = 0; i < fileCount; ++i) {
        const FileInfo &f = files[i];
        const bool isOpen = (f.len == kLenOpen);
        uint8_t name[kNameSize];
        nameToBytes(f.name.c_str(), f.name.size(), name);
        buildEntryImage(img, name, isOpen ? kStateOpen : kStateClosed,
                        f.recovered ? kFlagRecovered : kFlagNormal, f.start,
                        f.len);
        const uint32_t addr =
            dirAddr + static_cast<uint32_t>(kHdrSize) +
            static_cast<uint32_t>(i) * static_cast<uint32_t>(kEntrySize);
        if (!_flashChip.write(addr, std::span<const uint8_t>(img, kEntrySize))) {
            return false;
        }
    }
    // Header last: a torn format keeps the old copy active (commit unset).
    uint8_t hdr[32];
    buildHeaderImage(hdr, seq, static_cast<uint16_t>(fileCount), kCommitDone);
    return _flashChip.write(dirAddr, std::span<const uint8_t>(hdr, kHdrSize));
}

// Parse one raw entry image into a FileInfo. Returns false for free/
// deleted/torn slots. If wantOpen is true, only OPEN entries match;
// otherwise only CLOSED entries match.
bool parseEntry(const uint8_t img[64], bool wantOpen,
                StorageManager::Filename &nameOut, uint32_t &startOut,
                uint32_t &lenOut, bool &recoveredOut) {
    if (isErased(img, kEntrySize)) {
        return false;
    }
    const uint8_t state = img[32];
    if (wantOpen ? (state != kStateOpen) : (state != kStateClosed)) {
        return false;
    }
    const uint8_t flags = img[33];
    if (flags != kFlagNormal && flags != kFlagRecovered) {
        return false;
    }
    const uint32_t start = getU32(img + 34);
    const uint32_t len = getU32(img + 38);
    if (entryCrcOpen(img, start) != img[42]) {
        return false; // torn startFile, ignore
    }
    if (!wantOpen && len != kLenOpen &&
        entryCrcClose(img, flags, start, len) != img[43]) {
        return false; // torn close, ignore
    }
    // Name: raw bytes up to first NUL (32-char names use the whole field).
    size_t n = 0;
    while (n < kNameSize && img[n] != 0x00) {
        ++n;
    }
    if (n == 0 || n > 32) {
        return false;
    }
    // Reject 0xFF padding leaking into the name (torn write).
    for (size_t i = 0; i < n; ++i) {
        if (img[i] == 0xFF) {
            return false;
        }
    }
    nameOut.assign(reinterpret_cast<const char *>(img), n);
    startOut = start;
    lenOut = len;
    recoveredOut = (flags == kFlagRecovered);
    return true;
}

int StorageManager::findFile(const Filename &filename) const {
    for (size_t i = 0; i < _files.size(); ++i) {
        if (_files[i].name == filename) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

int StorageManager::findFreeSlot() {
    const uint32_t slots =
        (_sectorSize - static_cast<uint32_t>(kHdrSize)) / static_cast<uint32_t>(kEntrySize);
    uint8_t img[64];
    for (uint32_t s = 0; s < slots; ++s) {
        if (!_flashChip.read(entryAddr(static_cast<int>(s)),
                             std::span(img, kEntrySize))) {
            return -1;
        }
        if (isErased(img, kEntrySize)) {
            return static_cast<int>(s);
        }
    }
    return -1;
}
// === PART2 END ===
void StorageManager::rebuildIdList() {
    _ids.clear();
    for (const FileInfo &f : _files) {
        if (!_ids.full()) {
            _ids.push_back(f.name);
        }
    }
}

void StorageManager::scanDir(uint32_t dirAddr, uint16_t seq) {
    (void)seq;
    _files.clear();
    _writePtr = _dataStart;
    uint8_t img[64];
    const uint32_t slots =
        (_sectorSize - static_cast<uint32_t>(kHdrSize)) / static_cast<uint32_t>(kEntrySize);
    for (uint32_t s = 0; s < slots; ++s) {
        const uint32_t addr = dirAddr + static_cast<uint32_t>(kHdrSize) +
                              s * static_cast<uint32_t>(kEntrySize);
        if (!_flashChip.read(addr, std::span(img, kEntrySize))) {
            continue;
        }
        Filename name;
        uint32_t start = 0;
        uint32_t len = 0;
        bool recovered = false;
        if (!parseEntry(img, false, name, start, len, recovered)) {
            continue; // free/deleted/torn/open handled by recoverOpenFiles
        }
        if (start < _dataStart || start >= _capacity) {
            continue;
        }
        if (len != kLenOpen && (len > _capacity || start + len > _capacity)) {
            continue;
        }
        if (_files.full() || findFile(name) >= 0) {
            continue;
        }
        _files.push_back(FileInfo{name, start, len, recovered});
        if (len != kLenOpen && start + len > _writePtr &&
            start + len <= _capacity) {
            _writePtr = start + len;
        }
    }
    rebuildIdList();
}

// Walk framed data from start, return framed byte count of the longest
// valid prefix. Stops at erased space, torn header, or CRC mismatch, so
// 0xFF payloads never confuse it.
uint32_t StorageManager::recoverLength(uint32_t start) {
    uint32_t len = 0;
    uint16_t expectSeq = 0;
    uint8_t fh[8];
    while (start + len + kFrameHdrSize <= _capacity) {
        if (!readVerifiedFrameHeader(start + len, _capacity, expectSeq, fh)) {
            break;
        }
        const uint16_t payLen = getU16(fh + 2);
        if (!verifyFramePayload(start + len, fh)) {
            break; // torn payload
        }
        len += static_cast<uint32_t>(kFrameHdrSize) + payLen;
        ++expectSeq;
    }
    return len;
}
// === PART3 END ===
// Shared frame accept rule: header must be intact (magic/len/seq) and the
// frame must fit inside capacityEnd. Payload CRC is checked separately by
// verifyFramePayload().
bool StorageManager::readVerifiedFrameHeader(uint32_t frameAddr,
                                             uint32_t capacityEnd,
                                             uint16_t expectSeq,
                                             uint8_t fhOut[8]) {
    if (frameAddr + kFrameHdrSize > capacityEnd) {
        return false;
    }
    if (!_flashChip.read(frameAddr, std::span(fhOut, kFrameHdrSize))) {
        return false;
    }
    if (isErased(fhOut, kFrameHdrSize)) {
        return false;
    }
    if (getU16(fhOut) != kFrameMagic) {
        return false;
    }
    const uint16_t payLen = getU16(fhOut + 2);
    const uint16_t seq = getU16(fhOut + 4);
    if (payLen == 0 || payLen > BUFFER_SIZE || seq != expectSeq) {
        return false;
    }
    if (frameAddr + kFrameHdrSize + payLen > capacityEnd) {
        return false;
    }
    return true;
}

bool StorageManager::verifyFramePayload(uint32_t frameAddr,
                                        const uint8_t fh[8]) {
    const uint16_t payLen = getU16(fh + 2);
    if (payLen == 0 || payLen > BUFFER_SIZE) {
        return false;
    }
    uint8_t hdrZero[8];
    memcpy(hdrZero, fh, 8);
    hdrZero[6] = 0;
    hdrZero[7] = 0;
    uint16_t crc = crc16(hdrZero, 8);
    std::array<uint8_t, 256> page{};
    uint32_t off = frameAddr + static_cast<uint32_t>(kFrameHdrSize);
    uint32_t left = payLen;
    while (left > 0) {
        const size_t chunk = left > page.size() ? page.size() : left;
        if (!_flashChip.read(off, std::span(page.data(), chunk))) {
            return false;
        }
        crc = crc16Update(crc, page.data(), chunk);
        off += static_cast<uint32_t>(chunk);
        left -= static_cast<uint32_t>(chunk);
    }
    const uint16_t want =
        static_cast<uint16_t>(fh[6]) | (static_cast<uint16_t>(fh[7]) << 8);
    return crc == want;
}
// In-place close: program the tail (len + crcClose) and clear the state
// bits OPEN->CLOSED. Erased bytes are FF so this only clears 1->0.
// Returns false if the slot moved (compaction raced) - caller rescans.
bool StorageManager::closeEntry(const Filename &filename, uint32_t start,
                                uint32_t len, bool recovered) {
    const uint32_t slots =
        (_sectorSize - static_cast<uint32_t>(kHdrSize)) / static_cast<uint32_t>(kEntrySize);
    uint8_t img[64];
    uint8_t want[kNameSize];
    nameToBytes(filename.c_str(), filename.size(), want);
    for (uint32_t s = 0; s < slots; ++s) {
        const uint32_t addr = entryAddr(static_cast<int>(s));
        if (!_flashChip.read(addr, std::span(img, kEntrySize))) {
            return false;
        }
        if (isErased(img, kEntrySize)) {
            continue;
        }
        if (memcmp(img, want, kNameSize) != 0) {
            continue;
        }
        if (img[32] != kStateOpen) {
            continue;
        }
        if (getU32(img + 34) != start) {
            continue;
        }
        if (entryCrcOpen(want, start) != img[42]) {
            continue;
        }
        const uint8_t flags = recovered ? kFlagRecovered : kFlagNormal;
        // Tail covers bytes 32..43: state/flags/start/len/crcOpen/crcClose.
        // Bytes 42 (crcOpen) is already programmed and can only clear bits,
        // so rewrite its current value to avoid corrupting it, then set 43.
        uint8_t tail[12];
        memset(tail, 0xFF, sizeof(tail));
        tail[0] = kStateClosed; // clears OPEN bits only
        tail[1] = flags;        // FF->FE or stays FF
        putU32(tail + 2, start); // same value, clears nothing new
        putU32(tail + 6, len);
        tail[10] = img[42]; // preserve programmed crcOpen (1->0 only)
        tail[11] = entryCrcClose(want, flags, start, len);
        if (!_flashChip.write(addr + 32, std::span<const uint8_t>(tail, sizeof(tail)))) {
            return false;
        }
        return true;
    }
    return false;
}

void StorageManager::recoverOpenFiles() {
    const uint32_t slots =
        (_sectorSize - static_cast<uint32_t>(kHdrSize)) / static_cast<uint32_t>(kEntrySize);
    uint8_t img[64];
    for (uint32_t s = 0; s < slots; ++s) {
        if (!_flashChip.read(entryAddr(static_cast<int>(s)),
                             std::span(img, kEntrySize))) {
            continue;
        }
        Filename name;
        uint32_t start = 0;
        uint32_t len = 0;
        bool recovered = false;
        if (!parseEntry(img, true, name, start, len, recovered)) {
            continue;
        }
        if (start < _dataStart || start >= _capacity) {
            continue;
        }
        const uint32_t flen = recoverLength(start);
        if (!closeEntry(name, start, flen, true)) {
            continue;
        }
        if (!_files.full() && findFile(name) < 0) {
            _files.push_back(FileInfo{name, start, flen, true});
        }
        if (start + flen > _writePtr && start + flen <= _capacity) {
            _writePtr = start + flen;
        }
    }
    _writePtr = alignUp(_writePtr, _sectorSize);
    if (_writePtr < _dataStart) {
        _writePtr = _dataStart;
    }
    _erasedUpTo = _writePtr;
    rebuildIdList();
}

bool StorageManager::compactDir() {
    FileInfo live[MAX_FILES];
    size_t n = 0;
    for (const FileInfo &f : _files) {
        if (n < MAX_FILES) {
            live[n++] = f;
        }
    }
    const uint32_t dst = inactiveDirAddr();
    const uint16_t seq = static_cast<uint16_t>(_activeSeq + 1);
    if (!formatDir(dst, live, n, seq)) {
        return false;
    }
    if (!_flashChip.eraseSector(activeDirAddr())) {
        return false; // new copy is already valid; old erase can retry later
    }
    _activeDir = (_activeDir == 0) ? 1 : 0;
    _activeSeq = seq;
    return true;
}

// Sector containing _writePtr must already be erased; erase lazily as the
// frontier advances. Never erases a sector that holds committed data.
bool StorageManager::ensureErased(uint32_t addr, uint32_t len) {
    if (!_mounted || len == 0 || addr < _dataStart || addr + len > _capacity) {
        return false;
    }
    const uint32_t first = (addr / _sectorSize) * _sectorSize;
    const uint32_t last =
        ((addr + len - 1) / _sectorSize) * _sectorSize;
    for (uint32_t s = first; s <= last; s += _sectorSize) {
        if (s < _erasedUpTo || s < _dataStart) {
            continue;
        }
        if (s == first && addr != s) {
            continue; // mid-sector append, bytes before addr are valid frames
        }
        if (!_flashChip.eraseSector(s)) {
            return false;
        }
        _erasedUpTo = s + _sectorSize;
    }
    return true;
}

// Erase exactly one not-yet-erased sector ahead of the write frontier so
// flushes rarely have to erase inline. Skipped during flight bursts by
// simply not calling update(). Clamped to a small lookahead so ground idle
// does not erase the whole chip (wear + boot-time stall).
void StorageManager::preEraseIdle() {
    if (!_mounted || _hasOpen) {
        return; // data content (and thus _erasedUpTo) may change on close
    }
    uint32_t s = _erasedUpTo;
    if (s < _dataStart) {
        s = _dataStart;
    }
    if (s + _sectorSize > _capacity) {
        return; // flash full, nothing to pre-erase
    }
    // Keep at most kPreEraseAhead sectors erased ahead of the write
    // frontier; _writePtr only moves on close, so this is stable while idle.
    constexpr uint32_t kPreEraseAhead = 2;
    const uint32_t frontier = alignUp(_writePtr, _sectorSize);
    if (s >= frontier + kPreEraseAhead * _sectorSize) {
        return;
    }
    if (s < frontier) {
        s = frontier;
        if (s + _sectorSize > _capacity) {
            return;
        }
    }
    if (!_flashChip.eraseSector(s)) {
        return; // retry next update(); don't advance _erasedUpTo
    }
    _erasedUpTo = s + _sectorSize;
}
// === PART4 END ===
bool StorageManager::flushFrame() {
    if (!_hasOpen || _bufUsed == 0) {
        return true;
    }
    const uint32_t total =
        static_cast<uint32_t>(kFrameHdrSize) + static_cast<uint32_t>(_bufUsed);
    if (_writePtr + total > _capacity) {
        return false;
    }
    if (!ensureErased(_writePtr, total)) {
        return false;
    }
    uint8_t fh[8];
    putU16(fh + 0, kFrameMagic);
    putU16(fh + 2, static_cast<uint16_t>(_bufUsed));
    putU16(fh + 4, _frameSeq);
    fh[6] = 0;
    fh[7] = 0;
    putU16(fh + 6, frameCrcRam(fh, _byteBuffer.data(), _bufUsed));
    if (!_flashChip.write(_writePtr, std::span<const uint8_t>(fh, 8))) {
        return false;
    }
    if (!_flashChip.write(_writePtr + static_cast<uint32_t>(kFrameHdrSize),
                          std::span<const uint8_t>(_byteBuffer.data(), _bufUsed))) {
        return false; // torn frame ignored on mount (CRC fails)
    }
    _writePtr += total;
    _openFile.len += total;
    ++_frameSeq;
    _bufUsed = 0;
    return true;
}

bool StorageManager::begin() {
    _sectorSize = _flashChip.getSectorSize();
    if (_sectorSize < kHdrSize + kEntrySize || _sectorSize == 0) {
        return false;
    }
    _capacity = _flashChip.getCapacity();
    if (_capacity < 2 * _sectorSize + _sectorSize || _capacity <= _sectorSize) {
        return false;
    }
    if (!_flashChip.isConnected()) {
        return false;
    }
    _dataStart = 2 * _sectorSize;
    _mounted = true;
    _bufUsed = 0;
    _hasOpen = false;
    _openFile = FileInfo{};
    _frameSeq = 0;

    uint8_t ha[32];
    uint8_t hb[32];
    const bool va = readHeader(0, ha);
    const bool vb = readHeader(_sectorSize, hb);
    if (!va && !vb) {
        if (!formatDir(0, nullptr, 0, 0)) {
            _mounted = false;
            return false;
        }
        _activeDir = 0;
        _activeSeq = 0;
        scanDir(0, 0);
        _writePtr = _dataStart;
        _erasedUpTo = _dataStart;
        return true;
    }
    const uint16_t sa = va ? getU16(ha + 6) : 0;
    const uint16_t sb = vb ? getU16(hb + 6) : 0;
    auto newer = [](uint16_t a, uint16_t b) {
        return static_cast<uint16_t>(a - b) < 0x8000 && a != b;
    };
    if (va && (!vb || !newer(sb, sa))) {
        _activeDir = 0;
        _activeSeq = sa;
        scanDir(0, sa);
    } else {
        _activeDir = 1;
        _activeSeq = sb;
        scanDir(_sectorSize, sb);
    }
    recoverOpenFiles();
    return true;
}
// === PART5 END ===
bool StorageManager::startFile(Filename filename) {
    if (!_mounted || _hasOpen || _files.full() || filename.empty()) {
        return false;
    }
    if (filename.size() > kNameSize) {
        return false;
    }
    if (findFile(filename) >= 0) {
        return false; // names are unique; delete first to reuse
    }
    int slot = findFreeSlot();
    if (slot < 0) {
        if (!compactDir()) {
            return false;
        }
        slot = findFreeSlot(); // compaction switched the active dir
        if (slot < 0) {
            return false;
        }
    }
    const uint32_t start = alignUp(_writePtr, _sectorSize);
    if (start >= _capacity) {
        return false;
    }
    uint8_t img[64];
    uint8_t name[kNameSize];
    nameToBytes(filename.c_str(), filename.size(), name);
    buildEntryImage(img, name, kStateOpen, kFlagNormal, start, kLenOpen);
    if (!_flashChip.write(entryAddr(slot),
                          std::span<const uint8_t>(img, kEntrySize))) {
        return false;
    }
    _hasOpen = true;
    _openFile = FileInfo{filename, start, 0, false};
    _frameSeq = 0;
    _bufUsed = 0;
    _writePtr = start;
    return true;
}

bool StorageManager::finishFile() {
    if (!_mounted || !_hasOpen) {
        return false;
    }
    while (_hasOpen && _bufUsed > 0) {
        if (!flushFrame()) {
            return false; // flash full/error; file stays open, retry later
        }
    }
    const Filename name = _openFile.name;
    const uint32_t start = _openFile.start;
    const uint32_t len = _openFile.len;
    if (!closeEntry(name, start, len, false)) {
        return false;
    }
    if (!_files.full() && findFile(name) < 0) {
        _files.push_back(FileInfo{name, start, len, false});
    }
    rebuildIdList();
    _hasOpen = false;
    _openFile = FileInfo{};
    _writePtr = alignUp(start + len, _sectorSize);
    return true;
}

std::optional<StorageManager::Filename> StorageManager::getCurrentFile() const {
    if (!_hasOpen) {
        return std::nullopt;
    }
    return _openFile.name;
}

bool StorageManager::isFileOpen() const {
    return _hasOpen;
}

uint32_t StorageManager::fileSizeFramed(const Filename &file) const {
    const int i = findFile(file);
    if (i < 0) {
        return 0;
    }
    return _files[static_cast<size_t>(i)].len;
}

uint32_t StorageManager::fileSizePayload(const Filename &file) {
    const int i = findFile(file);
    if (i < 0) {
        return 0;
    }
    const FileInfo &f = _files[static_cast<size_t>(i)];
    uint32_t payload = 0;
    uint32_t addr = f.start;
    const uint32_t end = f.start + f.len;
    uint16_t expectSeq = 0;
    uint8_t fh[8];
    // Read-only walk, but the Arduino SPI flash driver mutates bus state, so
    // _flashChip is non-const and this stays a non-const method.
    while (addr + kFrameHdrSize <= end) {
        if (!readVerifiedFrameHeader(addr, end, expectSeq, fh)) {
            break;
        }
        const uint16_t payLen = getU16(fh + 2);
        if (!verifyFramePayload(addr, fh)) {
            break;
        }
        payload += payLen;
        addr += static_cast<uint32_t>(kFrameHdrSize) + payLen;
        ++expectSeq;
    }
    return payload;
}

uint32_t StorageManager::freeSpace() const {
    if (!_mounted || _writePtr >= _capacity) {
        return 0;
    }
    return _capacity - _writePtr;
}

uint32_t StorageManager::usedSpace() const {
    if (!_mounted || _writePtr < _dataStart) {
        return 0;
    }
    return _writePtr - _dataStart;
}

bool StorageManager::flush() {
    if (!_mounted || !_hasOpen) {
        return false;
    }
    while (_bufUsed > 0) {
        if (!flushFrame()) {
            return false; // buffer retained for retry
        }
    }
    return true;
}

void StorageManager::discardBuffered() {
    _bufUsed = 0;
}

const etl::ivector<StorageManager::Filename> &StorageManager::listFiles() const {
    return _ids;
}

bool StorageManager::deleteFile(Filename file) {
    if (!_mounted) {
        return false;
    }
    // Deleting the open file aborts the log: discard the uncommitted RAM
    // tail (LogManager re-reports it as dropped on the next log) and
    // tombstone the committed prefix. Deleting any other file only
    // tombstones: arena space is append-only and reclaimed by
    // deleteAllFiles() on the ground.
    const bool deletingOpen = _hasOpen && _openFile.name == file;
    uint32_t openStart = 0;
    if (deletingOpen) {
        openStart = _openFile.start;
        _hasOpen = false;
        _openFile = FileInfo{};
        _bufUsed = 0;
        // The open file always occupies the arena tail ([openStart,
        // _writePtr)), since no other file can be created while one is open.
        // Rewind so the next log reuses the space immediately instead of
        // waiting for deleteAllFiles(). Older files are NOT rewound: their
        // arena bytes stay allocated (append-only design).
        _writePtr = openStart;
        if (_erasedUpTo > _writePtr) {
            _erasedUpTo = _writePtr;
        }
    }
    const uint32_t slots =
        (_sectorSize - static_cast<uint32_t>(kHdrSize)) / static_cast<uint32_t>(kEntrySize);
    uint8_t img[64];
    uint8_t want[kNameSize];
    nameToBytes(file.c_str(), file.size(), want);
    for (uint32_t s = 0; s < slots; ++s) {
        const uint32_t addr = entryAddr(static_cast<int>(s));
        if (!_flashChip.read(addr, std::span(img, kEntrySize))) {
            return false;
        }
        if (isErased(img, kEntrySize) || memcmp(img, want, kNameSize) != 0) {
            continue;
        }
        if (img[32] == kStateDeleted || img[32] == 0xFF) {
            return false;
        }
        if (img[32] == kStateOpen) {
            // Crash leftover: close it as recovered, then tombstone below.
            Filename nm;
            uint32_t st = 0;
            uint32_t ln = 0;
            bool rec = false;
            if (!parseEntry(img, true, nm, st, ln, rec)) {
                return false;
            }
            const uint32_t flen = recoverLength(st);
            if (!closeEntry(nm, st, flen, true)) {
                return false;
            }
            if (!_flashChip.read(addr, std::span(img, kEntrySize))) {
                return false;
            }
        }
        // Tombstone: clear CLOSED->DELETED bits in place (no erase, no move).
        uint8_t tomb = kStateDeleted;
        if (!_flashChip.write(addr + 32, std::span<const uint8_t>(&tomb, 1))) {
            return false;
        }
        const int i = findFile(file);
        if (i >= 0) {
            _files.erase(_files.begin() + static_cast<ptrdiff_t>(i));
            rebuildIdList();
        }
        return true;
    }
    return false;
}

void StorageManager::deleteAllFiles() {
    if (!_mounted) {
        return;
    }
    _hasOpen = false;
    _openFile = FileInfo{};
    _bufUsed = 0;
    formatDir(0, nullptr, 0, static_cast<uint16_t>(_activeSeq + 1));
    _flashChip.eraseSector(_sectorSize);
    _activeDir = 0;
    ++_activeSeq;
    _files.clear();
    _ids.clear();
    _writePtr = _dataStart;
    _erasedUpTo = _dataStart;
}

size_t StorageManager::readFile(Filename file, std::uint32_t offset,
                                std::span<std::uint8_t> output) {
    if (!_mounted || output.empty()) {
        return 0;
    }
    const int i = findFile(file);
    if (i < 0) {
        return 0;
    }
    const FileInfo &f = _files[static_cast<size_t>(i)];
    size_t copied = 0;
    uint32_t addr = f.start;
    const uint32_t end = f.start + f.len;
    uint32_t skip = offset; // payload bytes to skip (de-framed space)
    uint16_t expectSeq = 0;
    uint8_t fh[8];
    std::array<uint8_t, 256> tmp{};
    while (addr + kFrameHdrSize <= end && copied < output.size()) {
        if (!readVerifiedFrameHeader(addr, end, expectSeq, fh)) {
            break; // corrupt/torn tail: same rule as recoverLength()
        }
        const uint16_t payLen = getU16(fh + 2);
        const uint32_t payAddr = addr + static_cast<uint32_t>(kFrameHdrSize);
        if (!verifyFramePayload(addr, fh)) {
            break; // bit-flip or torn payload: stop, don't return bad data
        }
        if (skip >= payLen) {
            skip -= payLen;
        } else {
            const uint32_t from = payAddr + skip;
            size_t want = payLen - skip;
            if (want > output.size() - copied) {
                want = output.size() - copied;
            }
            size_t got = 0;
            while (got < want) {
                const size_t chunk =
                    (want - got) > tmp.size() ? tmp.size() : (want - got);
                if (!_flashChip.read(from + static_cast<uint32_t>(got),
                                     std::span(tmp.data(), chunk))) {
                    break;
                }
                memcpy(output.data() + copied + got, tmp.data(), chunk);
                got += chunk;
            }
            copied += got;
            skip = 0;
            if (got < want) {
                break;
            }
        }
        ++expectSeq;
        addr = payAddr + payLen;
    }
    return copied;
}

void StorageManager::update() {
    if (!_mounted) {
        return;
    }
    if (_hasOpen && _bufUsed > 0) {
        flushFrame(); // at most one frame; may do one sector erase inline
        return;
    }
    preEraseIdle(); // at most one sector erase, only when idle
}

StorageManager::WriteResult StorageManager::write(std::span<const uint8_t> data) {
    if (!_mounted || !_hasOpen) {
        return WriteResult::NoOpenFile;
    }
    size_t off = 0;
    while (off < data.size()) {
        const size_t room = BUFFER_SIZE - _bufUsed;
        if (room == 0) {
            if (!flushFrame()) {
                // Keep staged bytes for retry after freeing space; mount
                // still re-derives length from committed frames only.
                return WriteResult::WriteError;
            }
            continue;
        }
        const size_t n = std::min(room, data.size() - off);
        memcpy(_byteBuffer.data() + _bufUsed, data.data() + off, n);
        _bufUsed += n;
        off += n;
        if (_bufUsed == BUFFER_SIZE) {
            if (!flushFrame()) {
                return WriteResult::WriteError;
            }
        }
    }
    return WriteResult::Ok;
}
// === PART6 END ===
