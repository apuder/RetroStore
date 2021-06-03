
#include <trs-io.h>
#include <unordered_map>
#include "serial.h"
#include "smb.h"
#include "posix.h"

#include "fileio.h"

#define TRS_FS_VERSION_MAJOR 1
#define TRS_FS_VERSION_MINOR 0

#define TRS_FS_MODULE_ID 4

TRS_FS* trs_fs = NULL;

const char* init_trs_fs() {
  if (trs_fs != NULL) {
    delete trs_fs;
  }
  //trs_fs = new TRS_FS_SMB();
  trs_fs = new TRS_FS_POSIX();
  return trs_fs->get_err_msg();
  //trs_fs = new TRS_FS_SERIAL();
}

const char* init_trs_fs(const char* url, const char* user, const char* passwd) {
  if (trs_fs != NULL) {
    delete trs_fs;
  }
  //trs_fs = new TRS_FS_SMB(url, user, passwd);
  trs_fs = new TRS_FS_POSIX();
  return trs_fs->get_err_msg();
  //trs_fs = new TRS_FS_SERIAL();
}

const char* get_smb_err_msg() {
  if (trs_fs == NULL) {
    return "SMB not connected";
  }
  return trs_fs->get_err_msg();
}

using namespace std;


class TrsFileSystemModule : public TrsIO {
public:
  TrsFileSystemModule(int id) : TrsIO(id) {
    //trs_fs = new TRS_FS_SMB();
    addCommand(static_cast<cmd_t>(&TrsFileSystemModule::doVersion), "BB");
    addCommand(static_cast<cmd_t>(&TrsFileSystemModule::doOpen), "SB");
    addCommand(static_cast<cmd_t>(&TrsFileSystemModule::doWrite), "BX");
    addCommand(static_cast<cmd_t>(&TrsFileSystemModule::doRead), "BL");
    addCommand(static_cast<cmd_t>(&TrsFileSystemModule::doClose), "B");
  }

  uint8_t clientVersionMajor;
  uint8_t clientVersionMinor;
  
  uint8_t nextFd = 0;
  unordered_map<uint8_t, FIL> fileMap;
  
public:
  void doVersion() {
    clientVersionMajor = B(0);
    clientVersionMinor = B(1);
    addByte(TRS_FS_VERSION_MAJOR);
    addByte(TRS_FS_VERSION_MINOR);
  }

  void doOpen() {
    const char* path = S(0);
    uint8_t mode = B(0);
    uint8_t fd = nextFd++;
    FRESULT result = f_open(&fileMap[fd], path, mode);
    addByte(result);
    if (result == FR_OK) {
      addByte(fd);
    } else {
      nextFd--;
    }
  }

  void doWrite() {
    FIL* fp = &fileMap[B(0)];
    uint32_t bufferLen = XL(0);
    uint8_t* buffer = X(0);
    UINT bw;
    FRESULT result = f_write(fp, buffer, bufferLen, &bw);
    addByte(result);
    if (result == FR_OK) {
      addLong(bw);
    }
  }

  void doRead() {
    FIL* fp = &fileMap[B(0)];
    uint32_t length = L(0);

    addByte(FR_OK);
    uint8_t* buf = startBlob32();
    if (length > getSendBufferFreeSize()) {
      length = getSendBufferFreeSize();
    }
    UINT br;
    FRESULT result = f_read(fp, buf, length, &br);
    if (result != FR_OK) {
      rewind();
      addByte(result);
    } else {
      skip(br);
      endBlob32();
    }
  }
  
  void doClose() {
    FIL* fp = &fileMap[B(0)];
    fileMap.erase(B(0));
    FRESULT result = f_close(fp);
    addByte(result);
  }
};

TrsFileSystemModule theTrsFileSystemModule(TRS_FS_MODULE_ID);
