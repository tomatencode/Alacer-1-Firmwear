#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "etl/string.h"
#include "etl/vector.h"

#include "hardwareIO/flash/FlashChip.hpp"

class StorageManager {
public:
    StorageManager(hardware::FlashChip &flashChip);

    using Filename = etl::string<32>;
    static constexpr size_t MAX_FILES = 63;

    // Mounts, recovers any torn-open file. Returns false when the flash
    // chip is not connected or geometry is unusable (not mounted).
    bool begin();

    bool startFile(Filename filename);
    bool finishFile();

    std::optional<Filename> getCurrentFile() const;
    bool isFileOpen() const;

    const etl::ivector<Filename> &listFiles() const;

    // De-framed (payload) size of a file, for download progress/end.
    // Returns 0 for unknown files and for the currently open file
    // (its length is still growing).
    uint32_t fileSizePayload(const Filename &file);
    // Framed (on-flash, incl. 8B frame headers) size. Same caveats.
    uint32_t fileSizeFramed(const Filename &file) const;

    uint32_t capacity() const { return _capacity; }
    uint32_t freeSpace() const;
    uint32_t usedSpace() const;

    // Bytes currently staged in RAM, not yet flushed to flash.
    size_t bufferedBytes() const { return _bufUsed; }
    // Best-effort flush of the staged frame. False = flash full/error,
    // buffer is retained so the caller can retry after freeing space.
    bool flush();
    // Drop staged bytes without writing (used when aborting a log).
    void discardBuffered();

    bool deleteFile(Filename file);
    void deleteAllFiles();

    // Copies de-framed payload bytes starting at payload offset.
    // CRC/seq-verified: stops before the first corrupt frame.
    // O(offset+copied): for sequential download call with increasing
    // offsets; for large files prefer small sequential chunks.
    size_t readFile(Filename file, std::uint32_t offset, std::span<std::uint8_t> output);

    // Only Ok / NoOpenFile / WriteError are returned today. BufferFull is
    // reserved: write() buffers in RAM and reports WriteError only when a
    // flush to flash fails (flash full or hardware error); buffered bytes
    // are retained for retry.
    enum class WriteResult : uint8_t { Ok, BufferFull, NoOpenFile, WriteError };
    WriteResult write(std::span<const uint8_t> data);

    // Flushes one frame / pre-erases one sector. May block on erase.
    // Call from idle/ground, not from a time-critical flight loop.
    void update();

    bool isMounted() const { return _mounted; }

private:
    struct FileInfo {
        Filename name{};
        uint32_t start = 0;
        uint32_t len = 0;
        bool recovered = false;
    };

    static constexpr size_t BUFFER_SIZE = 1024;

    uint32_t activeDirAddr() const;
    uint32_t inactiveDirAddr() const;
    static uint32_t alignUp(uint32_t v, uint32_t align);
    uint32_t entryAddr(int slot) const;
    bool readHeader(uint32_t dirAddr, uint8_t img[32]);
    bool formatDir(uint32_t dirAddr, const FileInfo *files, size_t fileCount,
                   uint16_t seq);
    int findFile(const Filename &filename) const;
    int findFreeSlot();
    void scanDir(uint32_t dirAddr, uint16_t seq);
    void recoverOpenFiles();
    uint32_t recoverLength(uint32_t start);
    bool closeEntry(const Filename &filename, uint32_t start, uint32_t len,
                    bool recovered);
    bool compactDir();
    bool ensureErased(uint32_t addr, uint32_t len);
    void preEraseIdle();
    bool flushFrame();
    void rebuildIdList();
    // Frame verification shared by recoverLength() and readFile(): same
    // accept rule, so a file that mounts is read back identically.
    // fhOut must point to 8 bytes.
    bool readVerifiedFrameHeader(uint32_t frameAddr, uint32_t capacityEnd,
                                 uint16_t expectSeq, uint8_t fhOut[8]);
    // Verifies the CRC16 of the payload streamed from flash. fh must be the
    // 8 header bytes as read (crc in bytes 6..7).
    bool verifyFramePayload(uint32_t frameAddr, const uint8_t fh[8]);

    hardware::FlashChip &_flashChip;

    bool _mounted = false;
    uint32_t _sectorSize = 4096;
    uint32_t _capacity = 0;
    uint32_t _dataStart = 0;
    int _activeDir = 0;
    uint16_t _activeSeq = 0;
    uint32_t _writePtr = 0;
    uint32_t _erasedUpTo = 0;

    bool _hasOpen = false;
    FileInfo _openFile{};
    uint16_t _frameSeq = 0;
    size_t _bufUsed = 0;
    std::array<uint8_t, BUFFER_SIZE> _byteBuffer{};

    etl::vector<FileInfo, MAX_FILES> _files{};
    etl::vector<Filename, MAX_FILES> _ids{};
};