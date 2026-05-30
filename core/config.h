#pragma once
#include <string>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <iostream>

class Config {
public:
	inline static std::unordered_map<std::string, std::string> configs;
	
	static void load(const std::string& config_file) {
		std::ifstream file(config_file);
		std::string line, k, v;
		while (std::getline(file, line)) {
			if (line.empty()) continue;

			std::stringstream ss(line);
			if (std::getline(ss, k, '=') && std::getline(ss, v)) {
				configs[k] = v;
				std::cout << "CONFIG: Found (" << k << ", " << v << ")\n";
			}
		}
	}

	static std::string get(const std::string& key) {
		if (configs.count(key)) {
			return configs[key];
		}

		std::cout << "Warning: " << key << " in config not found. Using empty string\n";
		return "";
	}
	
	static int getInt(const std::string& key) {
		if (configs.count(key)) {
			return std::stoi(configs[key]);
		}

		std::cout << "Warning: " << key << " in config not found. Using 0\n";
		return 0;
	}

	static void set(const std::string& key, const std::string& value) {
		configs[key] = value;
	}
};
