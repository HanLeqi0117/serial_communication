/*-----------------------------------------------
 * 	serial_communication2.cpp
 * <Last Update>	R2/10/1
 * <version>		v2.0
 * <editor>  YAMAMOTO KYOUSUKE
 *
 * <MEMO>
 * シリアル通信用プログラム
 *  serial_comunication.cppを元に作成したシリアル通信用プログラム
 * 変更点
 * 改行コードごとにtiny powerからの返信をpublish
 * 書き込みむための関数をwrite_someからasync_write_someに変更
 * 周期的に書き込むように変更
 * ---------------------------------------------*/

#include <boost/asio.hpp>
#include "serial_communication2.h"
using namespace serial;

std::vector<std::string> split(std::string str, std::string separator) {
    if (separator == "") return {str};
    std::vector<std::string> result;
    std::string tstr = str + separator;
    long l = tstr.length(), sl = separator.length();
    std::string::size_type pos = 0, prev = 0;
    for (;pos < l && (pos = tstr.find(separator, pos)) != std::string::npos; prev = (pos += sl)) {
        result.emplace_back(tstr, prev, pos - prev);
    }
    return result;
}
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
    //
	//ros関係
	ros::NodeHandle n; //ono
	ros::NodeHandle private_nh("~");
	private_nh.param("command_interval", command_timer_interval, 0.02);
    //tiny_delim = "\r\n";
	sub_ = n.subscribe("/serial_send", 10, &serial::SerialPort::Callback, this); //ono
	command_timer_ = n.createTimer(ros::Duration(command_timer_interval), &serial::SerialPort::timer_callback, this);
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

    //@@@@@@@@ボーレー卜の設定@@@@@@@@@@@
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
	impl->u_serial_port->async_read_some(boost::asio::buffer(rBuffer, 1024),boost::bind(&SerialPort::read_ok, this, _1, _2));
    //impl->u_serial_port->async_write_some(boost::asio::buffer(rBuffer, 1024),boost::bind(&SerialPort::write_ok, this, _1, _2));
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
	//impl->u_serial_port->async_write_some(boost::asio::buffer(str, n),boost::bind(&SerialPort::write_ok, this, _1, _2));
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
		std::cerr << "async_read_some() Error = " << e << std::endl;
		return;
	}
    //std::cout<<"read ok"<<std::endl;
	// 受信処理
	std::string str(rBuffer, rBuffer + size);

	// 更新処理
	notifyAll(str);

	// 読み込みが完了したので、再度設定
	//impl->u_serial_port->async_read_some(boost::asio::buffer(rBuffer, 1024),boost::bind(&SerialPort::read_ok, this, _1, _2));
	impl->u_serial_port->async_read_some(boost::asio::buffer(rBuffer, 1024),boost::bind(&SerialPort::read_ok, this, _1, _2));
}

void serial::SerialPort::write_ok(const boost::system::error_code &e, size_t size)
{
	if (!is_connect_)
		return;
	if (e)
	{
		std::cerr << "async_write_some() Error = " << e << std::endl;
		return;
	}
}

void serial::SerialPort::Callback(const std_msgs::String::ConstPtr &str_msg)
{
	//std::cout<<"receive to serial"<<std::endl;
    boost::mutex::scoped_lock look(send_sync_);
	//send_state_.wait(send_sync_);
    que_string.push(str_msg->data);
	if(que_string.size()>20){
	  que_string.pop();
	  //std::cout<<"que is too many"<<std::endl;
	}
	//send_state_.notify_all();
	return;
}
void serial::SerialPort::timer_callback(const ros::TimerEvent& e)
{
    boost::mutex::scoped_lock look(send_sync_);
	//send_state_.wait(send_sync_);
	if(que_string.empty()){
	  //send_state_.notify_all();
	  return;
	}
	std::string que_data = que_string.front();
	//std::cout<<"sending msg to tiny power:"<<que_data<<std::endl;
	que_string.pop();
	send(que_data);
	//send_state_.notify_all();
	return;
}

//非同期通信用のオブザーバーパターン
class Observer : public SerialObserver
{
public:
	Observer()
	{
		//ros関係
		ros::NodeHandle n; //ono
		tiny_delim = "\r\n";
		pub_ = n.advertise<std_msgs::String>("serial_receive", 10); //ono
		raw_pub_ = n.advertise<std_msgs::String>("raw_serial_receive", 10); //ono
        rest_data = "";
	}
	virtual ~Observer() {}
	ros::Publisher pub_;
	ros::Publisher raw_pub_;
    std::string rest_data;
	std::string tiny_delim;
protected:
	void notify(const std::string &str);
};

void Observer::notify(const std::string &str)
{
	//std::cout << "----tiny power from msg----"<<std::endl;
	std::cout<<str<<std::endl;
	if(str.empty()) {
		//std::cout <<"msg is empty:"<<std::endl;
		return;
	}
	std_msgs::String str_msg;
	str_msg.data = str;
	raw_pub_.publish(str_msg);
	rest_data.append(str);
	bool last_crlf_in = false;
	bool crlf_in = false;
	//std::vector<std::string> v;
	//std::cout << str << std::endl;
	int lastCRLF = rest_data.rfind(tiny_delim);
    if (lastCRLF == std::string::npos) {
		//std::cout << "CRLF is not Found" <<std::endl;
		std::cout << rest_data <<std::endl;
		return;
    }else {
		crlf_in = true;
        //std::cout << "CRLFは"<<lastCRLF << "番目にあります"<<std::endl; 
		//std::cout << "受信したメッセージは" << rest_data.size() << "文字です"<<std::endl;
		if(rest_data.size() - 3 == lastCRLF)
		{
			//std::cout <<"CRLF is last:"<<rest_data<<std::endl;
			//最後に改行コードが入っている
			last_crlf_in = true;
		}
		//std::cout <<"CRLF is not last:"<<rest_data<<std::endl;
    }
	std::vector<std::string> v = split(rest_data, tiny_delim);
	if(last_crlf_in){
		//改行コードが最後に入っている場合
    	for(auto itr = v.begin(); itr != v.end(); ++itr) {
        	str_msg.data = *itr;
			if(! str_msg.data.empty()) pub_.publish(str_msg);
    	}
		return;
	}else{
		//改行コードが最後に入っていない場合
    	for(auto itr = v.begin(); itr != v.end()-1; ++itr) {
        	str_msg.data = *itr;
			if(! str_msg.data.empty()) pub_.publish(str_msg);
    	}
		rest_data = v.back();
		//std::cout << "送信しなかったメッセージは" <<std::endl;
		//std::cout << rest_data <<std::endl;
		//std::cout << "です"<<std::endl;
		return;
	} 
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "serial_communication2");

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
