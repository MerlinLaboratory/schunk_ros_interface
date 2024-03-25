#include <iostream>
#include <boost/asio.hpp>

int main() {

    // Create io_context
    boost::asio::io_context io_context;

    // Create socket
    boost::asio::ip::tcp::socket socket(io_context);

    // Connect to robot gripper
    socket.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string("192.168.125.160"), 44818));

    // Send data
    // std::string data = "Hello, gripper!";
    // socket.write_some(boost::asio::buffer(data));

    // Receive data
    char reply[1024];
    std::cout << "Reply is aaaaa" << std::endl;

    size_t reply_length = socket.read_some(boost::asio::buffer(reply, 10));
    std::cout << "Reply is: " << std::string(reply, reply_length) << std::endl;

    return 0;
}