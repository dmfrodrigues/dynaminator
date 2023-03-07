#include <cstring>
#include <iostream>

#include "script/utils.hpp"

using namespace std;

int main() {
    cout << "Content-type: text/html\n\n";

    if (strcmp(getenv("REQUEST_METHOD"), "GET") == 0) {
        auto getParams = GetParams();
        for(const auto &p: getParams){
            cout << p.first << " => " << p.second << "\n";
        }
    } else {
        cout << "request is not supported";
    }

    cout << "Hello world!" << endl;

    return 0;
}
