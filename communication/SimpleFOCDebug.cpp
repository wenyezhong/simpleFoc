#include "SimpleFOCDebug.h"

#ifndef SIMPLEFOC_DISABLE_DEBUG

void testSimpleFOCDebug(void)
{
    SimpleFOCDebug::enable();
    SIMPLEFOC_DEBUG("hello SIMPLEFOC_DEBUG test\r\n");
}

Print* SimpleFOCDebug::_debugPrint = NULL;
// Print* SimpleFOCDebug::_debugPrint = NULL;

void SimpleFOCDebug::enable(Print* debugPrint) {
    _debugPrint = debugPrint;
}


void SimpleFOCDebug::println(int val) {
    if (_debugPrint != NULL) {
        _debugPrint->println(val);
    }
}

void SimpleFOCDebug::println(float val) {
    if (_debugPrint != NULL) {
        _debugPrint->println(val);
    }
}



void SimpleFOCDebug::println(const char* str) {
    if (_debugPrint != NULL) {
        _debugPrint->println(str);
    }
}

/* void SimpleFOCDebug::println(const __FlashStringHelper* str) {
    if (_debugPrint != NULL) {
        _debugPrint->println(str);
    }
} */

void SimpleFOCDebug::println(const char* str, float val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}

/* void SimpleFOCDebug::println(const __FlashStringHelper* str, float val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
} */

void SimpleFOCDebug::println(const char* str, int val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
}

/* void SimpleFOCDebug::println(const __FlashStringHelper* str, int val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
        _debugPrint->println(val);
    }
} */


void SimpleFOCDebug::print(const char* str) {
    if (_debugPrint != NULL) {
        _debugPrint->print(str);
    }
}




void SimpleFOCDebug::print(int val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(val);
    }
}


void SimpleFOCDebug::print(float val) {
    if (_debugPrint != NULL) {
        _debugPrint->print(val);
    }
}


void SimpleFOCDebug::println() {
    if (_debugPrint != NULL) {
        _debugPrint->println();
    }
}

#endif