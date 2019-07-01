#include <utility>
#include <iostream>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>


std::sig_atomic_t signal_value;

void set_signal(int value){
    signal_value = static_cast<decltype(signal_value)>(value);
}
void init_system_signals(struct sigaction& action){
    action.sa_handler = set_signal;
    ::sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    ::sigaction(SIGINT, &action, nullptr);
    ::sigaction(SIGTERM, &action, nullptr);
    ::sigaction(SIGQUIT, &action, nullptr);
    ::sigaction(SIGHUP, &action, nullptr);
    signal_value = 0;
}

class ODriveSerial {
    using ReadCallback = std::function<void(std::string &&)>;

public:
    ODriveSerial(unsigned int baud, const std::string &device, ReadCallback cb) : io_buffer_(), cb_(cb) {
        io_service_ = std::make_shared<boost::asio::io_service>();
        serial_port_ = std::make_unique<boost::asio::serial_port>(*io_service_, device);
        if (!serial_port_->is_open()) {
            std::__throw_runtime_error("could not open serial");
        }
        boost::asio::serial_port_base::baud_rate baud_option(baud);
        serial_port_->set_option(baud_option);
        boost::thread t(boost::bind(&ODriveSerial::start_thread, this));
    }

    void write_async(std::string &&msg) {
        boost::asio::async_write(*serial_port_, boost::asio::buffer(msg),
                                 boost::bind(&ODriveSerial::write_async_complete, this,
                                             boost::asio::placeholders::error));
    }


    void terminate() {
        io_service_->stop();
        serial_port_->close();
    }


private:
    void start_thread(){
        start_reading();
        io_service_->run();
    }

    void start_reading() {
        serial_port_->async_read_some(boost::asio::buffer(io_buffer_, max_read_length),
                                      boost::bind(&ODriveSerial::read_async_complete, this,
                                                  boost::asio::placeholders::error,
                                                  boost::asio::placeholders::bytes_transferred));
    }

    void read_async_complete(const boost::system::error_code &error, size_t bytes_transferred) {
        if (!error) {
            for (auto i = 0; i < bytes_transferred; ++i) {
                if ((io_buffer_[i] != '\n') && ((io_buffer_[i] != '\r'))) {
                    read_buffer_.push_back(io_buffer_[i]);
                } else if(!read_buffer_.empty()) {
                    cb_(std::string(
                            std::accumulate(read_buffer_.begin(), read_buffer_.end(), std::string())));
                    read_buffer_.clear();
                }
            }
        }
        start_reading();
    }

    void write_async_complete(const boost::system::error_code &error) {
        if (error) {
            std::cout << "write error" << std::endl;
        }
    }


    static const int max_read_length = 512;
    std::unique_ptr<boost::asio::serial_port> serial_port_;
    std::shared_ptr<boost::asio::io_service> io_service_;
    ReadCallback cb_;
    char io_buffer_[max_read_length];
    boost::circular_buffer<char> read_buffer_{max_read_length};
};

void read_callback(std::string &&feedback) {
    std::cout << "fb:" + feedback << std::endl;
}

int main() {
    ODriveSerial serial = ODriveSerial(115200, "/dev/ttyJ2", read_callback);
    struct sigaction sa{};
    init_system_signals(sa);

    while (0 == signal_value){
        serial.write_async("f 0\r");
        boost::this_thread::sleep_for(boost::chrono::milliseconds(100));
    }

    serial.terminate();
    std::cout<<"shutting down"<<std::endl;

    return 0;
}