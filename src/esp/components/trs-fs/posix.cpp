
#include "trs-fs.h"
#include "posix.h"
#include "storage.h"
#include "io.h"

#include <string.h>
#include <assert.h>

#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <dirent.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>



TRS_FS_POSIX::TRS_FS_POSIX() {
  mount = "/sdcard";
  err_msg = NULL;
}

void TRS_FS_POSIX::f_log(const char* msg) {
  printf("TRS_FS_POSIX: %s\n", msg);
}
  
FRESULT TRS_FS_POSIX::f_open (
                            FIL* fp,           /* [OUT] Pointer to the file object structure */
                            const TCHAR* path, /* [IN] File name */
                            BYTE mode          /* [IN] Mode flags */
                            ) {
  char* abs_path;
  const char* m;
  
  switch(mode) {
  case FA_READ:
    m = "r";
    break;
  case FA_READ | FA_WRITE:
    m = "r+";
    break;
  case FA_CREATE_ALWAYS | FA_WRITE:
    m = "w";
    break;
  case FA_CREATE_ALWAYS | FA_WRITE | FA_READ:
    m = "w+";
    break;
  case FA_OPEN_APPEND | FA_WRITE:
    m = "a";
    break;
  case FA_OPEN_APPEND | FA_WRITE | FA_READ:
    m = "a+";
    break;
  case  FA_CREATE_NEW | FA_WRITE:
    m = "w";
    break;
  case FA_CREATE_NEW | FA_WRITE | FA_READ:
    m = "w+";
    break;
  default:
    assert(0);
  }

  asprintf(&abs_path, "%s/%s", mount, path);
  printf("Opening: %s\n", abs_path);
  fp->f = fopen(abs_path, m);
  printf("----\n");
  free(abs_path);
  return (fp->f == NULL) ? FR_NO_FILE : FR_OK;
}

FRESULT TRS_FS_POSIX::f_opendir (
                               DIR_* dp,           /* [OUT] Pointer to the directory object structure */
                               const TCHAR* path  /* [IN] Directory name */
                               ) {
  char* abs_path;

  if ((strcmp(path, ".") == 0) || (strcmp(path, "/") == 0)) {
    path = "";
  }
  asprintf(&abs_path, "%s/%s", mount, path);
  dp->dir = opendir(abs_path);
  free(abs_path);
  return (dp->dir != NULL) ? FR_OK : FR_DISK_ERR;
}

FRESULT TRS_FS_POSIX::f_write (
                             FIL* fp,          /* [IN] Pointer to the file object structure */
                             const void* buff, /* [IN] Pointer to the data to be written */
                             UINT btw,         /* [IN] Number of bytes to write */
                             UINT* bw          /* [OUT] Pointer to the variable to return number of bytes written */
                             ) {
  int _bw = fwrite(buff, 1, btw, (FILE*) fp->f);
  *bw = _bw;
  return (_bw >= 0) ? FR_OK : FR_DISK_ERR;
}

FRESULT TRS_FS_POSIX::f_read (
                            FIL* fp,     /* [IN] File object */
                            void* buff,  /* [OUT] Buffer to store read data */
                            UINT btr,    /* [IN] Number of bytes to read */
                            UINT* br     /* [OUT] Number of bytes read */
                            ) {
  int _br = fread(buff, 1, btr, (FILE*) fp->f);
  *br = _br;
  return (_br >= 0) ? FR_OK : FR_DISK_ERR;
}

FRESULT TRS_FS_POSIX::f_readdir (
                               DIR_* dp,      /* [IN] Directory object */
                               FILINFO* fno  /* [OUT] File information structure */
                                  ) {
  while (1) {
    struct dirent* entry = readdir((DIR*) dp->dir);
    if (entry == NULL) {
      closedir((DIR*) dp->dir);
      fno->fname[0] = '\0';
      break;
    }
    if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0) {
      continue;
    }
    if (strlen(entry->d_name) > 12) {
      continue;
    }
    strcpy(fno->fname, entry->d_name);
    f_stat(fno->fname, fno);
    //fno->fattrib = 1;
    break;
  }
  return FR_OK;
}

FSIZE_t TRS_FS_POSIX::f_tell (
                            FIL* fp   /* [IN] File object */
                            ) {
  assert(0);
}

FRESULT TRS_FS_POSIX::f_sync (
                            FIL* fp     /* [IN] File object */
                            ) {
  assert(0);
}

FRESULT TRS_FS_POSIX::f_lseek (
                             FIL*    fp,  /* [IN] File object */
                             FSIZE_t ofs  /* [IN] File read/write pointer */
                             ) {
  fseek((FILE*) fp->f, ofs, SEEK_SET);
  return FR_OK;
}
  
FRESULT TRS_FS_POSIX::f_close (
                             FIL* fp     /* [IN] Pointer to the file object */
                             ) {
  fclose((FILE*) fp->f);
  return FR_OK;
}

FRESULT TRS_FS_POSIX::f_unlink (
                              const TCHAR* path  /* [IN] Object name */
                              ) {
  char* abs_path;
  asprintf(&abs_path, "%s/%s", mount, path);
  int r = unlink(abs_path);
  free(abs_path);
  return r ? FR_NO_FILE : FR_OK;
}

FRESULT TRS_FS_POSIX::f_stat (
                            const TCHAR* path,  /* [IN] Object name */
                            FILINFO* fno        /* [OUT] FILINFO structure */
                            ) {
  char* abs_path;
  struct stat s;
  asprintf(&abs_path, "%s/%s", mount, path);
  printf("STAT: %s\n", abs_path);
  int r = stat(abs_path, &s);
  free(abs_path);
  if (r != 0) {
    return FR_NO_FILE;
  }
  strcpy(fno->fname, path);
  fno->fsize = s.st_size;
  printf("SIZE: %d\n", fno->fsize);
  fno->fattrib = 1; //XXX
  return FR_OK;
}
