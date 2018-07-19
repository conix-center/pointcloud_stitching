#include <iostream>
#include <cstdlib>
#include <cstring>
#include "mqtt/client.h"

const std::string SERVER_ADDRESS("tcp://192.168.0.113:1883");
const std::string CLIENT_ID("central_subscriber");
const std::string TOPIC("cameras");

const int QOS = 1;

bool try_reconnect(mqtt::client& cli)
{
	constexpr int N_ATTEMPT = 30;

	for (int i = 0; i < N_ATTEMPT && !cli.is_connected(); ++i) {
		try {
			cli.reconnect();
			return true;
		}
		catch (const mqtt::exception&) {
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}
	return false;
}

int main(int argc, char** argv) {
	mqtt::connect_options conn_opts;
	conn_opts.set_keep_alive_interval(20);
	conn_opts.set_clean_session(true);
	conn_opts.set_automatic_reconnect(true);

	mqtt::client client(SERVER_ADDRESS, CLIENT_ID);

	try {
		client.connect(conn_opts);
		client.subscribe(TOPIC, QOS);

		for (int i = 0; i < 40; i++) {
			auto msg = client.consume_message();

			if (!msg) {
				if (!client.is_connected()) {
					std::cout << "Lost connection. Attempting to reconnect" << std::endl;
					if (try_reconnect(client)) {
						client.subscribe(TOPIC, QOS);
						std::cout << "Reconnected" << std::endl;
						continue;
					}
					else {
						std::cout << "Reconnect failed." << std::endl;
						break;
					}
				}
				else
					break;
			}

			std::cout << msg->get_topic() << ": " << msg->to_string() << std::endl;
		}

		client.disconnect();
	}
	catch (const mqtt::exception& exc) {
		std::cerr << exc.what() << std::endl;
		return 1;
	}

	return 0;
}