#include <iostream>
#include <cstdlib>
#include <cstring>
#include "mqtt/client.h"

const std::string SERVER_ADDRESS("tcp://192.168.0.113:1883");
const std::string CLIENT_ID("camera_1");
const std::string TOPIC("cameras");

const std::string CAMERA_ID("CAMERA_ID: 1");
const std::string IP_ADDR("IP_ADDR: 192.168.0.115");
const std::string PORT("PORT: 8000");
const std::string POSITION("POSITION: 2.001, 3.114, 4.194");
const std::string MESSAGE = CAMERA_ID + ", " + IP_ADDR + ", " + PORT + ", " + POSITION;

const int QOS = 1;

class user_callback : public virtual mqtt::callback
{
	void connection_lost(const std::string& cause) override {
		std::cout << "\nConnection lost" << std::endl;
		if (!cause.empty())
			std::cout << "\tcause: " << cause << std::endl;
	}

	void delivery_complete(mqtt::delivery_token_ptr tok) override {
		std::cout << "[Delivery complete for token: "
			<< (tok ? tok->get_message_id() : -1) << "]" << std::endl;
	}

public:
};

int main(int argc, char** argv) {
	mqtt::connect_options conn_opts;
	conn_opts.set_keep_alive_interval(20);
	conn_opts.set_clean_session(true);

	mqtt::client client(SERVER_ADDRESS, CLIENT_ID);
	user_callback cb;
	client.set_callback(cb);

	try {
		client.connect(conn_opts);

		for (int i = 0; i < 20; i++) {
			client.publish(mqtt::message(TOPIC, MESSAGE, QOS, false));
			// client.publish(mqtt::message(TOPIC, CAMERA_ID, QOS, false));
			// client.publish(mqtt::message(TOPIC, IP_ADDR, QOS, false));
			// client.publish(mqtt::message(TOPIC, PORT, QOS, false));
			// client.publish(mqtt::message(TOPIC, POSITION, QOS, false));
		}

		client.disconnect();
	}
	catch (const mqtt::exception& exc) {
		std::cerr << exc.what() << std::endl;
		return 1;
	}

	return 0;
}