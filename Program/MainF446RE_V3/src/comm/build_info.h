#ifndef BUILD_INFO_H_
#define BUILD_INFO_H_

#include <stdint.h>

// ビルドの識別 (Makefile が build_info.o を作るたびに埋め込む。build_info.c)。
// 上位は VERSION の build_epoch を記録 (diag/link) に残し、どのファームで走ったかを後から分かるようにする

// ビルドした時刻 (UNIX 秒)。Makefile を通さないビルドでは 0
extern const uint32_t g_build_epoch;
// ビルドした git のコミット (短いハッシュ。作業ツリーに未コミットの変更があれば末尾に "+dirty")
extern const char g_build_git[];

#endif /* BUILD_INFO_H_ */
