
#include "trs-fs.h"

#define CHECK() if(trs_fs == NULL || trs_fs->get_err_msg() != NULL) return FR_NOT_READY;

void f_log(const char* format, ...) {
#if 1
  char* msg;
  va_list args;
  va_start(args, format);
  vasprintf(&msg, format, args);
  va_end(args);
  trs_fs->f_log(msg);
  free(msg);
#endif
}

FRESULT f_open (
                FIL* fp,           /* [OUT] Pointer to the file object structure */
                const TCHAR* path, /* [IN] File name */
                BYTE mode          /* [IN] Mode flags */
                ) {
  f_log("f_open: %s", path);
  CHECK();
  FRESULT err = trs_fs->f_open(fp, path, mode);
  f_log("err: %d", err);
  return err;
}

FRESULT f_opendir (
                   DIR_* dp,           /* [OUT] Pointer to the directory object structure */
                   const TCHAR* path  /* [IN] Directory name */
                   ) {
  f_log("f_opendir: %s", path);
  CHECK();
  FRESULT err = trs_fs->f_opendir(dp, path);
  f_log("err: %d", err);
  return err;
}

FRESULT f_write (
                 FIL* fp,          /* [IN] Pointer to the file object structure */
                 const void* buff, /* [IN] Pointer to the data to be written */
                 UINT btw,         /* [IN] Number of bytes to write */
                 UINT* bw          /* [OUT] Pointer to the variable to FRESULT err = number of bytes written */
                 ) {
  f_log("f_write");
  CHECK();
  FRESULT err = trs_fs->f_write(fp, buff, btw, bw);
  f_log("err: %d", err);
  return err;
}

FRESULT f_read (
                FIL* fp,     /* [IN] File object */
                void* buff,  /* [OUT] Buffer to store read data */
                UINT btr,    /* [IN] Number of bytes to read */
                UINT* br     /* [OUT] Number of bytes read */
                ) {
  f_log("f_read");
  CHECK();
  FRESULT err = trs_fs->f_read(fp, buff, btr, br);
  f_log("err: %d", err);
  return err;
}

FRESULT f_readdir (
                   DIR_* dp,      /* [IN] Directory object */
                   FILINFO* fno  /* [OUT] File information structure */
                   ) {
  f_log("f_readdir");
  CHECK();
  FRESULT err = trs_fs->f_readdir(dp, fno);
  f_log("fn : %s", fno->fname);
  f_log("err: %d", err);
  return err;
}

FSIZE_t f_tell (
                FIL* fp   /* [IN] File object */
                ) {
  f_log("f_tell");
  CHECK();
  FSIZE_t size = trs_fs->f_tell(fp);
  f_log("size: %d", size);
  return size;
}

FRESULT f_sync (
                FIL* fp     /* [IN] File object */
                ) {
  f_log("f_sync");
  CHECK();
  FRESULT err = trs_fs->f_sync(fp);
  f_log("err: %d", err);
  return err;
}

FRESULT f_lseek (
                 FIL*    fp,  /* [IN] File object */
                 FSIZE_t ofs  /* [IN] File read/write pointer */
                 ) {
  f_log("f_lseek");
  CHECK();
  FRESULT err = trs_fs->f_lseek(fp, ofs);
  f_log("err: %d", err);
  return err;
}

FRESULT f_close (
                 FIL* fp     /* [IN] Pointer to the file object */
                 ) {
  f_log("f_close");
  CHECK();
  FRESULT err = trs_fs->f_close(fp);
  f_log("err: %d", err);
  return err;
}

FRESULT f_unlink (
                  const TCHAR* path  /* [IN] Object name */
                  ) {
  f_log("f_unlink: %s", path);
  CHECK();
  FRESULT err = trs_fs->f_unlink(path);
  f_log("err: %d", err);
  return err;
}

FRESULT f_stat (
                const TCHAR* path,  /* [IN] Object name */
                FILINFO* fno        /* [OUT] FILINFO structure */
                ) {
  f_log("f_stat: %s", path);
  CHECK();
  FRESULT err = trs_fs->f_stat(path, fno);
  f_log("err: %d", err);
  return err;
}
