#ifndef APP_H_
#define APP_H_

// 全モジュールのインスタンスを所有し、初期化順序とメインループを組み立てる層。
// 個々の機能は src/ 以下の各モジュールが持ち、ここには制御の中身を書かない。

/**
 * @brief ペリフェラルと全モジュールを初期化する。main() から一度だけ呼ぶこと。
 */
void Setup();

/**
 * @brief メインループ。戻らない。Setup() の後に呼ぶこと。
 */
void MainApp();

#endif  // APP_H_
