#ifndef TRS_FS_POSIX_H
#define TRS_FS_POSIX_H


#include "trs-fs.h"


class TRS_FS_POSIX : virtual public TRS_FS {
private:
  const char* mount;

public:
  TRS_FS_POSIX();
  void f_log(const char* msg);
  FRESULT f_open (
                  FIL* fp,           /* [OUT] Pointer to the file object structure */
                  const TCHAR* path, /* [IN] File name */
                  BYTE mode          /* [IN] Mode flags */
                  );
  FRESULT f_opendir (
                     DIR_* dp,           /* [OUT] Pointer to the directory object structure */
                     const TCHAR* path  /* [IN] Directory name */
                     );
  FRESULT f_write (
                   FIL* fp,          /* [IN] Pointer to the file object structure */
                   const void* buff, /* [IN] Pointer to the data to be written */
                   UINT btw,         /* [IN] Number of bytes to write */
                   UINT* bw          /* [OUT] Pointer to the variable to return number of bytes written */
                   );
  FRESULT f_read (
                  FIL* fp,     /* [IN] File object */
                  void* buff,  /* [OUT] Buffer to store read data */
                  UINT btr,    /* [IN] Number of bytes to read */
                  UINT* br     /* [OUT] Number of bytes read */
                  );
  FRESULT f_readdir (
                     DIR_* dp,      /* [IN] Directory object */
                     FILINFO* fno  /* [OUT] File information structure */
                     );
  FSIZE_t f_tell (
                  FIL* fp   /* [IN] File object */
                  );
  FRESULT f_sync (
                  FIL* fp     /* [IN] File object */
                  );
  FRESULT f_lseek (
                   FIL*    fp,  /* [IN] File object */
                   FSIZE_t ofs  /* [IN] File read/write pointer */
                   );
  FRESULT f_close (
                   FIL* fp     /* [IN] Pointer to the file object */
                   );
  FRESULT f_unlink (
                    const TCHAR* path  /* [IN] Object name */
                    );
  FRESULT f_stat (
                  const TCHAR* path,  /* [IN] Object name */
                  FILINFO* fno        /* [OUT] FILINFO structure */
                  );

};

#endif
