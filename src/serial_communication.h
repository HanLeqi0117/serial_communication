/*-----------------------------------------------
 * 	serial_communication.h
 * <Last Update>	H29/08/25
 * <version>		v2.0
 * <editor> Takafumi_ONO
 * <MEMO>
 * シリアル通信用ヘッダー
 * asioはここで読み込まないように
 * ---------------------------------------------*/

#ifndef USERIAL_PORT_H_
#define USERIAL_PORT_H_

#include <stdio.h>
#include <string>
#include <list>
#include <iostream>
#include <algorithm>
#include <queue>
#include <vector>

#include <boost/foreach.hpp>

#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/make_shared.hpp>

//ros用
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace boost{ namespace system{ class error_code; }; };

namespace serial
{
	/**
	* @brief：シリアルポートのオプションを設定するためのEnumクラス群
	*/
	// ボーレート設定用
	class BaudRate {
	public:
		enum {
			BR2400 = 2400,
			BR4800 = 4800,
			BR9600 = 9600,
			BR19200 = 19200,
			BR38400 = 38400,
			BR57600 = 57600,
			BR115200 = 115200
		};
	};

	// パリティ設定用
	class Parity {
	public:
		enum {
			None = 0,
			Odd,
			Even
		};
	};

	// ストップビット設定用
	class StopBits {
	public:
		enum {
			One = 0,
			OnePointFive,
			Two
		};
	};

	// データのバイトサイズ設定用
	class ByteSize{
	public:
		enum{
			B7 = 7,
			B8 = 8
		};
	};

	// フローコントロール用の設定
	class FlowControl{
	public:
		enum{
			None = 0,
			Hardware,
			Software
		};
	};

	class SerialPort;
	/**
	* @brief  : observerを使ってコールバックを実装するためのインターフェースクラス
	*/
	class SerialObserver
	{
		friend SerialPort;
	public:
		SerialObserver(){}
		virtual ~SerialObserver(){}

	protected:
		virtual void notify(const std::string& str) = 0;

	};

	/**
	* @brief    : boostを使ったシリアル通信クラス
	*             イベントで動くようにしたのため
	*           　通信に余分なリソースをさく必要がない
	* 			　送信側の成功判定が諸事情でOFFになっているけど
	* 			　気にしないで．voidに直していいよ
	*/
	class SerialPort
	{
		// コンストラクタ、デストラクタ------------------------
	public:
		SerialPort();
		virtual ~SerialPort();

		// 操作------------------------------------------------
	public:
		/*
		* @brief        : ポートのオープン
		* @param[in]    : comポート
		* @param[in]    : 1バイトのビット数
		* @param[in]    : パリティを指定
		* @param[in]    : ストップビット指定
		* @return       : 成功判定
		*/
		bool open(
			const std::string& com = "COM1",
			int baudrate = BaudRate::BR19200,
			int bytesize = ByteSize::B8,
			int parity = Parity::None,
			int stopbits = StopBits::One,
			int flowcontrol = FlowControl::None
			);

		/**
		* @brief    : ポートのクローズ
		* @return   : 成功判定
		*/
		bool close();

		/**
		* @brief     : 文字列を送信する関数
		* @param[in] : 送信する文字列
		*/
		bool send(const std::string& s);
		bool u_send(const std::string& s);
		/**
		* @brief     : 文字列を送信する関数
		* @param[in] : 送信する文字
		*/
		bool send(char c);

		/**
		* @brief     : 文字列を送信する関数
		* @param[in] : 送信する文字列
		* @param[in] : 文字列のサイズ
		*/
		bool send(const char *c, int size);

		/**
		* @brief     : observerを追加する関数
		* @param[in] : observerクラス
		* @return    : 成功判定
		*/
		bool attach(SerialObserver* ob);

		/**
		* @brief    : observerを削除する関数
		* @pram[in] : observerクラス
		* @return   : 成功判定
		*/
		bool detach(SerialObserver* ob);

		/**
		* @brief      : データを受信する関数
		* @param[out] : 取得データ
		* @return     : 残りのデータ数
		*/
		bool receive(std::string& str, double timeout);

		/**
		* @brief    : 接続確認
		* @return   : 接続状況
		*/
		bool isConnect() const { return is_connect_; }

		//ros
		//ros::Publisher pub_;
		ros::Subscriber sub_;
		void Callback(const std_msgs::String::ConstPtr& vel_msg);

    //ポートが開いているかどうかのフラグ
		bool open_flag;

	private:
		// 更新関数
		virtual void notifyAll(const std::string& str);

		// データ書き込み
		bool write(const char* str, int n);
		//連続送信用
		bool u_write(const char* str, int n);
		//実際の書き込み
		bool u_write_strand(const char* str, int n);

		// 属性------------------------------------------------
	private:
		class serial_impl;
		boost::shared_ptr<serial_impl> impl;

		// 受信用バッファ
		char rBuffer[1024];
		int readSize;

		void read_ok(const boost::system::error_code& e, size_t size);
		bool is_connect_;

		// 最新版の受信データ
		std::string readData;

	};
};


#endif
