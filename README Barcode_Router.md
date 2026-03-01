README

This project implements a barcode recognition-based path selection system to simulate a car automatically selecting a driving route after recognizing a barcode. The system reads the barcode-path mapping stored in an SQLite database, queries the database after scanning a barcode, and outputs the corresponding path result (PATH1–PATH4). This is a test version, currently using simulated barcode data for functional verification, primarily implementing the logical flow from "barcode recognition" to "path decision-making."

The system consists of three parts: a path control core module (BarcodeRouter class), an SQLite database (routes.db), and the main program entry point. Upon program startup, the barcode-path mapping relationship in the database is automatically loaded and stored in a hash table for fast lookup. When the scanning thread simulates reading a barcode, the system calls a matching function to determine the corresponding path and outputs movement instructions through a callback function.

The current version is a functional verification test version; the barcode data is generated internally by the program. Future versions will integrate with real barcode readers to achieve hardware input and real-time path decision-making.
