#pragma once

#include <Arduino.h>
#include <array>
#include <stdarg.h>
#include <string.h>

template <size_t N> class Table {
  public:
    Table(std::array<const char*, N> columnNames, const char* rowFormat) {
        this->columnNames = columnNames;
        this->rowFormat = rowFormat;

        for (int i = 0; i < this->columnNames.size(); i++) {
            if (i > 0) {
                this->seperator += '-';
            }

            const char* columnName = this->columnNames[i];
            size_t length = strlen(columnName);

            for (int k = 0; k < length; k++) {
                this->seperator += '-';
            }
            this->seperator += "-+";
        }
    }

    void printHeader() {
        Serial.println(this->seperator);

        Serial.print("| ");
        for (int i = 0; i < this->columnNames.size(); i++) {
            if (i > 0) {
                Serial.print(" ");
            }

            const char* columnName = this->columnNames[i];
            Serial.printf("%s |", columnName);
        }
        Serial.println();

        Serial.println(this->seperator);
    }

    void printRow(...) {
        char buffer[256];
        va_list args;
        va_start(args, N);
        vsnprintf(buffer, sizeof(buffer), this->rowFormat, args);
        Serial.print(buffer);
        Serial.println(this->seperator);
        va_end(args);
    }

  private:
    std::array<const char*, N> columnNames;
    String seperator = "+-";
    const char* rowFormat;
};
