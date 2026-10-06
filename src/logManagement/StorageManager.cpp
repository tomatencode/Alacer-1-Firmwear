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
    if (len > kNameSize) {
        len = kNameSize;
    }
    memcpy(out, name, len);
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
    std::array<uint8_t, 256> page{};
    while (start + len + kFrameHdrSize <= _capacity) {
        if (!_flashChip.read(start + len, std::span(fh, kFrameHdrSize))) {
            break;
        }
        if (isErased(fh, kFrameHdrSize)) {
            break;
        }
        if (getU16(fh) != kFrameMagic) {
            break;
        }
        const uint16_t payLen = getU16(fh + 2);
        const uint16_t seq = getU16(fh + 4);
        if (payLen == 0 || payLen > BUFFER_SIZE || seq != expectSeq) {
            break;
        }
        const uint32_t frameEnd = start + len + kFrameHdrSize + payLen;
        if (frameEnd > _capacity) {
            break;
        }
        uint16_t crc = 0xFFFF;
        uint8_t hdrZero[8];
        memcpy(hdrZero, fh, 8);
        hdrZero[6] = 0;
        hdrZero[7] = 0;
        for (size_t i = 0; i < 8; ++i) {
            crc ^= static_cast<uint16_t>(hdrZero[i]) << 8;
            for (int b = 0; b < 8; ++b) {
                crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                     : static_cast<uint16_t>(crc << 1);
            }
        }
        uint32_t off = start + len + static_cast<uint32_t>(kFrameHdrSize);
        uint32_t left = payLen;
        bool ok = true;
        while (left > 0) {
            const size_t chunk = left > page.size() ? page.size() : left;
            if (!_flashChip.read(off, std::span(page.data(), chunk))) {
                ok = false;
                break;
            }
            for (size_t i = 0; i < chunk; ++i) {
                crc ^= static_cast<uint16_t>(page[i]) << 8;
                for (int b = 0; b < 8; ++b) {
                    crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                         : static_cast<uint16_t>(crc << 1);
                }
            }
            off += static_cast<uint32_t>(chunk);
            left -= static_cast<uint32_t>(chunk);
        }
        if (!ok) {
            break;
        }
        if (crc != (static_cast<uint16_t>(fh[6]) | (static_cast<uint16_t>(fh[7]) << 8))) {
            break; // torn frame
        }
        len = frameEnd - start;
        ++expectSeq;
    }
    return len;
}
// === PART3 END ===
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
        uint8_t tail[12];
        memset(tail, 0xFF, sizeof(tail));
        tail[0] = kStateClosed; // clears OPEN bits only
        tail[1] = flags;        // FF->FE or stays FF
        putU32(tail + 2, start); // same value, clears nothing new
        putU32(tail + 6, len);
        tail[10] = entryCrcClose(want, flags, start, len);
        if (!_flashChip.write(addr + 32, std::span<const uint8_t>(tail, 10))) {
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
// simply not calling update().
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
    _flashChip.eraseSector(s);
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
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < 8; ++i) {
        crc ^= static_cast<uint16_t>(fh[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    for (size_t i = 0; i < _bufUsed; ++i) {
        crc ^= static_cast<uint16_t>(_byteBuffer[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    putU16(fh + 6, crc);
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

void StorageManager::begin() {
    _sectorSize = _flashChip.getSectorSize();
    if (_sectorSize < kHdrSize + kEntrySize || _sectorSize == 0) {
        _sectorSize = 4096;
    }
    _capacity = _flashChip.getCapacity();
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
        formatDir(0, nullptr, 0, 0);
        _activeDir = 0;
        _activeSeq = 0;
        scanDir(0, 0);
        _writePtr = _dataStart;
        _erasedUpTo = _dataStart;
        return;
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
            return false;
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

const etl::ivector<StorageManager::Filename> &StorageManager::listFiles() const {
    return _ids;
}

bool StorageManager::deleteFile(Filename file) {
    if (!_mounted) {
        return false;
    }
    // Close the open file first if it is the one being deleted.
    if (_hasOpen && _openFile.name == file) {
        _hasOpen = false;
        _openFile = FileInfo{};
        _bufUsed = 0;
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
            const size_t idx = static_cast<size_t>(i);
            _files.erase(_files.begin() + static_cast<ptrdiff_t>(idx));
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
    uint8_t fh[8];
    std::array<uint8_t, 256> tmp{};
    while (addr + kFrameHdrSize <= end && copied < output.size()) {
        if (!_flashChip.read(addr, std::span(fh, kFrameHdrSize))) {
            break;
        }
        if (getU16(fh) != kFrameMagic) {
            break;
        }
        const uint16_t payLen = getU16(fh + 2);
        if (payLen == 0 || addr + kFrameHdrSize + payLen > end) {
            break;
        }
        const uint32_t payAddr = addr + static_cast<uint32_t>(kFrameHdrSize);
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
                _bufUsed = 0; // drop torn tail; mount re-derives length
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
                _bufUsed = 0;
                return WriteResult::WriteError;
            }
        }
    }
    return WriteResult::Ok;
}
// === PART6 END ===
