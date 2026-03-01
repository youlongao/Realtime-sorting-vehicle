#ifndef BARCODE_ROUTER_H
#define BARCODE_ROUTER_H

#include <string>
#include <unordered_map>
#include <functional>
#include <thread>
#include <atomic>
#include "sqlite3.h"

enum class RouteType {
    Path1,
    Path2,
    Path3,
    Path4,
    UNKNOWN
};

using RouteCallback = std::function<void(RouteType)>;

class BarcodeRouter {
public:
    BarcodeRouter();
    ~BarcodeRouter();

    bool initialize();
    void registerCallback(RouteCallback callback);

private:
    std::unordered_map<std::string, RouteType> m_routeTable;
    RouteCallback m_callback;

    std::thread m_scanThread;
    std::atomic<bool> m_running;

    void loadRoutesFromDatabase();
    void scanThread();
    RouteType evaluateBarcode(const std::string& code);
};

#endif