#include "barcode_router.h"
#include <iostream>
#include <chrono>

BarcodeRouter::BarcodeRouter() : m_running(false) {}

BarcodeRouter::~BarcodeRouter() {
    m_running = false;
    if (m_scanThread.joinable())
        m_scanThread.join();
}

bool BarcodeRouter::initialize() {

    loadRoutesFromDatabase();   

    m_running = true;
    m_scanThread = std::thread(&BarcodeRouter::scanThread, this);

    return true;
}

void BarcodeRouter::registerCallback(RouteCallback callback) {
    m_callback = callback;
}

void BarcodeRouter::loadRoutesFromDatabase() {

    sqlite3* db;
    sqlite3_stmt* stmt;

    if (sqlite3_open("routes.db", &db) != SQLITE_OK) {
        std::cout << "Cannot open database\n";
        return;
    }

    const char* query = "SELECT barcode, route FROM routes;";

    if (sqlite3_prepare_v2(db, query, -1, &stmt, nullptr) != SQLITE_OK) {
        std::cout << "Failed to prepare query\n";
        sqlite3_close(db);
        return;
    }

    while (sqlite3_step(stmt) == SQLITE_ROW) {

        std::string barcode =
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, 0));

        std::string route =
            reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));

        if (route == "Path1")
            m_routeTable[barcode] = RouteType::Path1;
        else if (route == "Path2")
            m_routeTable[barcode] = RouteType::Path2;
        else if (route == "Path3")
            m_routeTable[barcode] = RouteType::Path3;
        else if (route == "Path4")
            m_routeTable[barcode] = RouteType::Path4;
    }

    sqlite3_finalize(stmt);
    sqlite3_close(db);

    std::cout << "Database loaded successfully\n";
}

RouteType BarcodeRouter::evaluateBarcode(const std::string& code) {

    auto it = m_routeTable.find(code);

    if (it != m_routeTable.end())
        return it->second;

    return RouteType::UNKNOWN;
}

void BarcodeRouter::scanThread() {

    std::string simulatedCodes[] = {
        "1234567",
        "2234567",
        "3234567",
        "4234567",
        "999999"
    };

    int index = 0;
    int total = sizeof(simulatedCodes) / sizeof(simulatedCodes[0]);

    while (m_running && index < total) {

        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::string code = simulatedCodes[index++];

        std::cout << "\n[Scanner Thread] Scanned: " << code << std::endl;

        RouteType route = evaluateBarcode(code);

        if (m_callback)
            m_callback(route);
    }
}