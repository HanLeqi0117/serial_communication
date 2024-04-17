/*-----------------------------------------------
 * 	serial_communication.cpp
 * <Last Update>	H29/08/27
 * <version>		v2.0
 * <editor> Takafumi ONO
 *
 * <MEMO>
 * シリアル通信用プログラム
 * ---------------------------------------------*/

 #include <boost/asio.hpp>
#include "serial_communication.h"
using namespace serial;

/// boostのインターフェースを隠蔽するためのクラス
class serial::SerialPort::serial_impl
{
public:
	serial_impl()
		: u_serial_port(NULL), br(boost::asio::serial_port_base::baud_rate(57600)), cs(boost::asio::serial_port_base::character_size(8)), fc(boost::asio::serial_port_base::flow_control::none), parity(boost::asio::serial_port_base::parity::none), sb(boost::asio::serial_port_base::stop_bits::one), com("COM1"), u_strand(io), work(io)
	{
	}

	virtual ~serial_impl()
	{
		io.stop();
		if (u_serial_port)
			delete u_serial_port;
		u_serial_port = NULL;
	}

	// 属性----------------------------------------------------
public:
	// shared_ptr
	std::vector<SerialObserver *> ptrList;

	// thread
	boost::thread ioThread;
	boost::condition recv_condition;
	boost::mutex recv_sync;

	// シリアルの設定系統
	boost::asio::io_service io;

	boost::asio::strand u_strand;
	boost::asio::io_service::work work;

	std::string com;
	boost::asio::serial_port *u_serial_port;
	boost::asio::serial_port_base::baud_rate br;
	boost::asio::serial_port_base::character_size cs;
	boost::asio::serial_port_base::flow_control::type fc;
	boost::asio::serial_port_base::parity::type parity;
	boost::asio::serial_port_base::stop_bits::type sb;
};

serial::SerialPort::SerialPort()
	: impl(new serial_impl()), is_connect_(false)
{

	//open_flag = false;//ono

	std::string Port = "/dev/kuaro/ttyUSBOkatech";

	if (open(Port))
	{
		std::cout << "シリアル通信開始" << std::endl;
		//open_flag = true;
	}
	else
	{
		std::cout << Port << "に接続できません。" << std::endl;
		exit(0);
	}

	//ros関係
	ros::NodeHandle n; //ono

	sub_ = n.subscribe("/serial_send", 10, &serial::SerialPort::Callback, this); //ono
																				 //pub_= n.advertise<std_msgs::String>("serial_receive",10);//ono
}

serial::SerialPort::~SerialPort()
{

	close();
	puts("good bye serial port");
	boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
}

/**
* @brief        : ポートのオープン
* @param[in]    : comポート
* @param[in]    : 1バイトのビット数
* @param[in]    : パリティを指定
* @param[in]    : ストップビット指定
* @return       : 成功判定
*/
bool serial::SerialPort::open(
	const std::string &com,
	int baudrate,
	int cs,
	int parity,
	int stopbits,
	int flow)
{
	if (is_connect_)
		return false;
	boost::system::error_code ret;

	// ポートのオープン
	impl->u_serial_port = new boost::asio::serial_port(impl->io);
	impl->u_serial_port->open(com, ret);

	if (ret)
	{
		std::cerr << "sresial_port open() error " << ret << std::endl;
		return false;
	}

	// 接続フラグ
	is_connect_ = true;

	// パリティ値の設定
	boost::asio::serial_port_base::parity::type parity_t = boost::asio::serial_port_base::parity::none;
	if (parity == Parity::Even)
		parity_t = boost::asio::serial_port_base::parity::even;
	else if (parity == Parity::Odd)
		parity_t = boost::asio::serial_port_base::parity::odd;

	// Stop Bists
	boost::asio::serial_port_base::stop_bits::type stopbit_t = boost::asio::serial_port_base::stop_bits::one;
	if (stopbits == StopBits::Two)
		stopbit_t = boost::asio::serial_port_base::stop_bits::two;
	else if (stopbits == StopBits::OnePointFive)
		stopbit_t = boost::asio::serial_port_base::stop_bits::onepointfive;

	// flow control
	boost::asio::serial_port_base::flow_control::type flow_t = boost::asio::serial_port_base::flow_control::none;
	if (flow == FlowControl::Hardware)
		flow_t = boost::asio::serial_port_base::flow_control::hardware;
	else if (flow == FlowControl::Software)
		flow_t = boost::asio::serial_port_base::flow_control::software;

	// 設定値の取得
	impl->com = com;
	impl->br = boost::asio::serial_port_base::baud_rate(115200);
	impl->cs = boost::asio::serial_port_base::character_size(cs);
	impl->parity = parity_t;
	impl->sb = stopbit_t;
	impl->fc = flow_t;

	impl->u_serial_port->set_option(impl->br);
	impl->u_serial_port->set_option(boost::asio::serial_port_base::parity(parity_t));
	impl->u_serial_port->set_option(boost::asio::serial_port_base::character_size(cs));
	impl->u_serial_port->set_option(boost::asio::serial_port_base::stop_bits(stopbit_t));
	impl->u_serial_port->set_option(boost::asio::serial_port_base::flow_control(flow_t));

	// 読み込み用の関数を設定
	impl->u_serial_port->async_read_some(
		boost::asio::buffer(rBuffer, 1024),
		boost::bind(&SerialPort::read_ok, this, _1, _2));

	// IOサービスの開始
	impl->ioThread = boost::thread(boost::bind(&boost::asio::io_service::run, &impl->io));

	return true;
}

/**
* @brier     : オブジェクトの登録を行う
* @param[in] : 登録を行うオブジェクト
* @return    : 成功判定
*/
bool serial::SerialPort::attach(SerialObserver *ob)
{
	std::vector<SerialObserver *>::iterator it = std::find(impl->ptrList.begin(), impl->ptrList.end(), ob);

	// 登録されていなかったら、オブザーバーを登録
	if (it != impl->ptrList.end())
		return false;
	impl->ptrList.push_back(ob);
	return true;
}

/**

* @brier     : オブジェクトの破棄を行う
* @param[in] : 破棄を行うオブジェクト
* @return    : 成功判定
*/
bool serial::SerialPort::detach(SerialObserver *ob)
{
	std::vector<SerialObserver *>::iterator it = std::find(impl->ptrList.begin(), impl->ptrList.end(), ob);

	// 登録されていなかったら、オブザーバーを登録
	if (it == impl->ptrList.end())
		return false;
	impl->ptrList.erase(it);
	return true;
}

/**
* @brief    : 状態の更新を通知する
* @param[in]: 受信文字列
*/
void serial::SerialPort::notifyAll(const std::string &str)
{
	// 全てのオブザーバーに通知
	BOOST_FOREACH (SerialObserver *ob, impl->ptrList)
		ob->notify(str);
	// コンディション解除
	boost::mutex::scoped_lock lk(impl->recv_sync);
	readData = str;
	//std::cout << "readData:"<<readData << std::endl;

	impl->recv_condition.notify_all();
}

/**
* @brief    : ポートのクローズ
* @return   : 成功判定
*/
bool serial::SerialPort::close()
{
	if (!is_connect_)
		return false;
	//impl->recv_condition.notify_all();
	impl->u_serial_port->close();
	is_connect_ = false;
	return true;
}

//多分使ってない
/*
* @brief    ： データリード
* @return   ： 成功判定
* @param[out]： 受信データ
* @param[in] : タイムアウト[ms]
*/
bool serial::SerialPort::receive(std::string &str, double timeout)
{

	// 接続判定
	if (!is_connect_)
		return false;

	boost::mutex::scoped_lock lk(impl->recv_sync);

	// 受信待ち
	boost::xtime xt;
	boost::xtime_get(&xt, boost::TIME_UTC_);
	xt.nsec += static_cast<int>(timeout * 1000.0 * 1000.0);

	// 受信待ち失敗
	if (!impl->recv_condition.timed_wait(lk, xt))
		return false;

	// 受信文字列を格納
	str = this->readData;
	return true;
}

/**
/* @brief   : 文字列の送信関数
/* @return  : 成功判定
*/
bool serial::SerialPort::send(const std::string &s)
{
	return write(s.c_str(), s.size());
}

bool serial::SerialPort::send(char c)
{
	return write(&c, 1);
}

bool serial::SerialPort::send(const char *c, int size)
{
	return write(c, size);
}

bool serial::SerialPort::u_send(const std::string &s)
{
	return u_write(s.c_str(), s.size());
}

bool serial::SerialPort::write(const char *str, int n)
{
	impl->io.post(impl->u_strand.wrap(boost::bind(&SerialPort::u_write_strand, this, str, n)));
	return true;
}

bool serial::SerialPort::u_write(const char *str, int n)
{
	impl->io.post(impl->u_strand.wrap(boost::bind(&SerialPort::u_write_strand, this, str, n)));
	return true;
}

bool serial::SerialPort::u_write_strand(const char *str, int n)
{
	if (!is_connect_)
		return false;
	boost::system::error_code ret;
	impl->u_serial_port->write_some(boost::asio::buffer(str, n), ret);
	if (ret)
	{
		std::cerr << "serial_port::write_some() return = " << ret << std::endl;
		return false;
	}
	return true;
}

void serial::SerialPort::read_ok(const boost::system::error_code &e, size_t size)
{
	if (!is_connect_)
		return;
	if (e)
	{
		std::cerr << "read_some() Error = " << e << std::endl;
		return;
	}

	// 受信処理
	std::string str(rBuffer, rBuffer + size);

	// 更新処理
	notifyAll(str);

	// 読み込みが完了したので、再度設定
	impl->u_serial_port->async_read_some(
		boost::asio::buffer(rBuffer, 1024),
		boost::bind(&SerialPort::read_ok, this, _1, _2));
}

void serial::SerialPort::Callback(const std_msgs::String::ConstPtr &str_msg)
{

	//if(!open_flag) return;
	//std::cout<<str_msg->data<<std::endl;
	// データ送信
	send(str_msg->data);
}

//非同期通信用のオブザーバーパターン
class Observer : public SerialObserver
{
public:
	Observer()
	{
		//ros関係
		ros::NodeHandle n; //ono

		pub_ = n.advertise<std_msgs::String>("serial_receive", 10); //ono
	}
	virtual ~Observer() {}
	ros::Publisher pub_;

protected:
	void notify(const std::string &str);
};

void Observer::notify(const std::string &str)
{
	//std::cout << str << std::endl;
	std_msgs::String str_msg;
	str_msg.data = str;
	pub_.publish(str_msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_communication");

	serial::SerialPort serial;

	//シリアル受信用（非同期）
	Observer ob;

	// オブザーバー登録
	serial.attach(&ob); //ono

	ros::spin();

	// オブザーバー登録解除
	serial.detach(&ob); //ono

	return 0;
}
