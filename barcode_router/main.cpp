#include "barcode_router.h"
#include <iostream>
#include <thread>

int main() {

    BarcodeRouter router;

    router.registerCallback(
        [](RouteType route) {

            switch(route) {
                case RouteType::Path1:
                    std::cout << "[Motion] Path1\n";
                    break;
                case RouteType::Path2:
                    std::cout << "[Motion] Path2\n";
                    break;
                case RouteType::Path3:
                    std::cout << "[Motion] Path3\n";
                    break;
                case RouteType::Path4:
                    std::cout << "[Motion] Path4\n";
                    break;
                default:
                    std::cout << "[Motion] Unknown barcode\n";
                    break;
            }
        }
    );

    if (!router.initialize()) {
        std::cout << "Initialization failed\n";
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(15));

    return 0;
}