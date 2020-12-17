#include <thread>
#include <vector>
#include <string>
#include <iostream>
#include <algorithm>

void f1(int n, std::string s) {
    std::cout << n << " " << s << std::endl;
}

void f2(int n, std::string s) {
    std::reverse(s.begin(), s.end());
    std::cout << n << " " << s << std::endl;
}

void input() {
    while(true) {
        char c = getchar();
        std::cout << "You pressed ASCI" << (int) c << std::endl;
    }
}


int main() {

    std::vector<std::thread> threads;

    threads.push_back(std::thread(f1, 0, "Hello"));
    threads.push_back(std::thread(f2, 1, "Hello"));
    threads.push_back(std::thread(f1, 2, "World"));
    threads.push_back(std::thread(f2, 3, "World"));
    threads.push_back(std::thread(f1, 4, "Akash"));
    threads.push_back(std::thread(f2, 5, "Akash"));
    threads.push_back(std::thread(f1, 6, "Mishra!"));
    threads.push_back(std::thread(f2, 7, "Mishra!"));

    for(auto &th : threads) {
        th.join();
    }

    return 0;
}