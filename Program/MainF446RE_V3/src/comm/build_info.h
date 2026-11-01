#ifndef BUILD_INFO_H_
#define BUILD_INFO_H_

#include <stdint.h>

// ビルドの識別 (Makefile が build_info.o を作るたびに埋め込む。build_info.c)。
// 修正しながら走らせると、記録からどのファームで走ったかが分からなかった (2026-10-31 の会場の試走:
// VERSION の build_epoch が常に 0)。上位は VERSION の build_epoch を記録 (diag/link) に残す

// ビルドした時刻 (UNIX 秒)。Makefile を通さないビルドでは 0
extern const uint32_t g_build_epoch;
// ビルドした git のコミット (短いハッシュ。作業ツリーに未コミットの変更があれば末尾に "+dirty")
extern const char g_build_git[];

#endif /* BUILD_INFO_H_ */
