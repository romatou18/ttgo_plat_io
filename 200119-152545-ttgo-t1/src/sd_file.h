#pragma once

#include "esp_common.h"
// #include "SPIFFS.h"
#include <SD.h>
#include <SPI.h>
#include <sd_defines.h>

// #define SD_MOSI 13
// #define SD_MISO 12
// #define SD_SCKL 2
// #define SD_CS 15
// #define SD_SPEED 26000000U //26 Mhz max on matrixed HSPI otherwise 27mhz

#define SD_MOSI 26
#define SD_MISO 25
#define SD_SCKL 32
#define SD_CS 33
#define SD_SPEED 4000000U //26 Mhz max on matrixed HSPI otherwise 27mhz

#define TAG "sd-manager";

#define MIN_SD_BYTES 5 * 1014 * 1024; //5MB

extern SPIClass spi_SD;

#define LOG_FILE_PATH "/logs/logFile.txt"

class SDMGT
{
private:
    String _filepath;
    File _file;
    bool _closed;
    uint8_t _flushCount;

    static const uint8_t FLUSH_LINE_LIMIT = 20;

public:
    ~SDMGT()
    {
        closeLogFile();
    }
    SDMGT(const char* filepath) 
    : _filepath(filepath), _closed(true), _flushCount(0)
    {
    }

    void openFile()
    {
        SDMGT::createDir(SD, "/logs");
        if (SD.exists(LOG_FILE_PATH))
        {
            ESP_LOGI(TAG, LOG_FILE_PATH " exists.");
            _file = SD.open(LOG_FILE_PATH, FILE_APPEND);
        }
        else
        {
            ESP_LOGI(TAG, LOG_FILE_PATH " doesn't exist.");
            _file = SD.open(LOG_FILE_PATH, FILE_WRITE);
        }
        _closed = false;
    }

    void logToFile(const char * line)
    {
        if(_closed)
        {
            openFile();
        }

        if(!_file.print(line)){
            ESP_LOGE(TAG,"- write failed");
        }
        ++_flushCount;
        if(_flushCount % FLUSH_LINE_LIMIT == 0)
        {
            _file.flush();
            _flushCount = 0;
        }
    }

    void readLogFileToSerial()
    {
         if (SD.exists(_filepath))
        {
            Serial.printf("%s %s", _filepath.c_str(), "exists.\n");
            _file = SD.open(_filepath, FILE_READ);
        }
        else
        {
            Serial.printf("%s %s", _filepath.c_str()," doesn't exist.\n");
        }

        if (!_file)
        {
            Serial.println("Failed to open file for reading");
            return;
        }

        Serial.print("Read from file: ");
        while (_file.available())
        {
            Serial.write(_file.read());
        }
        _file.close();
        _closed = true;
    }

    void closeLogFile()
    {
        if(!_closed)
        {
            _file.flush();
            _file.close();
            _closed = true;
        }
    }

    static
    uint64_t get_free_space_bytes()
    {
        return (SD.totalBytes() - SD.usedBytes());
    }

    static
    bool check_free_space_bytes()
    {
        return get_free_space_bytes() > MIN_SD_BYTES;
    }
    static
    uint64_t card_size_mb()
    {
        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        ESP_LOGI(TAG, "SD Card Size: %lluMB\n", cardSize);
        return cardSize;
    }
    static
    void reader_spi_setup(SDMGT& m)
    {
        // (14,2,15,13);
        spi_SD.begin(SD_SCKL, SD_MISO, SD_MOSI, SD_CS); //CLK,MISO,MOIS,SS
        ESP_LOGI(TAG, "SPI SD Init ok.");
        espDelay(100);
        ESP_LOGI(TAG, "Initializing SD card...");

        // see if the card is present and can be initialized:
        if (!SD.begin(SD_CS, spi_SD, SD_SPEED))
        {
            
            // don't do anything more:
            bool ok = false;
            while (!ok)
            {
                if (!SD.begin(SD_CS, spi_SD, SD_SPEED))
                {
                    ESP_LOGE(TAG, "SD Card mount failed!");
                }
                else
                {
                    ok = true;
                    ESP_LOGI(TAG, "SD Card mount success!");
                    break;
                }
                espDelay(2000);
            }
        }

        ESP_LOGI(TAG, "card initialized. checking free space...");
        if (!check_free_space_bytes())
        {
            ESP_LOGE(TAG, "SD Card full!");
            // don't do anything more:
            while (1)
                ;
        }
        m.openFile();
       
        // open a new file and immediately close it:
        ESP_LOGI(TAG, "reader_spi_setup() done");
    }

    static
    bool test_exists_dir(fs::FS &fs, const char *dirname)
    {
        File dir = fs.open(dirname);
        if (!dir)
        {
            ESP_LOGE(TAG, "Failed to open directory");
            return false;
        }
        if (!dir.isDirectory())
        {
            ESP_LOGE(TAG, "Not a directory");
            return false;
        }
        return true;
    }

    static
    void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
    {
        ESP_LOGI(TAG, "Listing directory: %s\n", dirname);

        File root = fs.open(dirname);
        if (!root)
        {
            ESP_LOGE(TAG, "Failed to open directory");
            return;
        }
        if (!root.isDirectory())
        {
            ESP_LOGE(TAG, "Not a directory");
            return;
        }

        File file = root.openNextFile();
        while (file)
        {
            if (file.isDirectory())
            {
                ESP_LOGI(TAG, " DIR : ");
                ESP_LOGI(TAG, file.name());
                if (levels)
                {
                    listDir(fs, file.name(), levels - 1);
                }
            }
            else
            {
                ESP_LOGI(TAG, " FILE: ");
                ESP_LOGI(TAG, file.name());
                ESP_LOGI(TAG, " SIZE: ");
                ESP_LOGI(file.size());
            }
            file = root.openNextFile();
        }
    }

    static
    bool createDir(fs::FS &fs, const char *path)
    {
        if (test_exists_dir(fs, path))
        {
            return true;
        }

        Serial.printf("Creating Dir: %s\n", path);
        if (fs.mkdir(path))
        {
            Serial.println("Dir created");
            return true;
        }
        Serial.println("mkdir failed");
        return false;
    }

    static
    bool removeDir(fs::FS &fs, const char *path)
    {
        Serial.printf("Removing Dir: %s\n", path);
        if (test_exists_dir(fs, path))
        {
            return true;
        }

        if (fs.rmdir(path))
        {
            Serial.println("Dir removed");
            return true;
        }
        Serial.println("rmdir failed");
        return false;
    }

    static
    void readFile(fs::FS &fs, const char *path)
    {
        Serial.printf("Reading file: %s\n", path);

        File file = fs.open(path);
        if (!file)
        {
            Serial.println("Failed to open file for reading");
            return;
        }

        Serial.print("Read from file: ");
        while (file.available())
        {
            Serial.write(file.read());
        }
        file.close();
    }

    static
    void writeFile(fs::FS &fs, const char *path, const char *message)
    {
        Serial.printf("Writing file: %s\n", path);

        File file = fs.open(path, FILE_WRITE);
        if (!file)
        {
            Serial.println("Failed to open file for writing");
            return;
        }
        if (file.print(message))
        {
            Serial.println("File written");
        }
        else
        {
            Serial.println("Write failed");
        }
        file.close();
    }

    static
    void appendFile(fs::FS &fs, const char *path, const char *message)
    {
        Serial.printf("Appending to file: %s\n", path);

        File file = fs.open(path, FILE_APPEND);
        if (!file)
        {
            Serial.println("Failed to open file for appending");
            return;
        }
        if (file.print(message))
        {
            Serial.println("Message appended");
        }
        else
        {
            Serial.println("Append failed");
        }
        file.close();
    }

    static
    void renameFile(fs::FS &fs, const char *path1, const char *path2)
    {
        Serial.printf("Renaming file %s to %s\n", path1, path2);
        if (fs.rename(path1, path2))
        {
            Serial.println("File renamed");
        }
        else
        {
            Serial.println("Rename failed");
        }
    }

    static
    void deleteFile(fs::FS &fs, const char *path)
    {
        Serial.printf("Deleting file: %s\n", path);
        if (fs.remove(path))
        {
            Serial.println("File deleted");
        }
        else
        {
            Serial.println("Delete failed");
        }
    }

    static
    void testFileIO(fs::FS &fs, const char *path)
    {
        File file = fs.open(path);
        static uint8_t buf[512];
        size_t len = 0;
        uint32_t start = millis();
        uint32_t end = start;
        if (file)
        {
            len = file.size();
            size_t flen = len;
            start = millis();
            while (len)
            {
                size_t toRead = len;
                if (toRead > 512)
                {
                    toRead = 512;
                }
                file.read(buf, toRead);
                len -= toRead;
            }
            end = millis() - start;
            Serial.printf("%u bytes read for %u ms\n", flen, end);
            file.close();
        }
        else
        {
            Serial.println("Failed to open file for reading");
        }

        file = fs.open(path, FILE_WRITE);
        if (!file)
        {
            Serial.println("Failed to open file for writing");
            return;
        }

        size_t i;
        start = millis();
        for (i = 0; i < 2048; i++)
        {
            file.write(buf, 512);
        }
        end = millis() - start;
        Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
        file.close();
    }

    static
    sdcard_type_t check_card_type()
    {
        // we'll use the initialization code from the utility libraries
        // since we're just testing if the card is working!
        uint8_t cardType = SD.cardType();
        ESP_LOGI(TAG, "Card type:         ");
        switch (cardType)
        {
        case CARD_NONE:
            ESP_LOGE(TAG, "No card found");
            break;
        case CARD_MMC:
            ESP_LOGI(TAG, "MMC");
            break;
        case CARD_SDHC:
            ESP_LOGI(TAG, "SDHC");
            break;
        default:
            ESP_LOGE(TAG, "CARD_UNKNOWN");
            break;
        }

        return SD.cardType();
    }

    static
    void test_sd_card()
    {
        uint64_t cardSize = card_size_mb();
        Serial.printf("SD Card Size: %lluMB\n", cardSize);
        Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
        Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
        Serial.printf("Used space: %lluMB\n", get_free_space_bytes() / (1024 * 1024));
        listDir(SD, "/", 0);
        createDir(SD, "/logs");
        createDir(SD, "/logs_2");
        listDir(SD, "/", 0);
        removeDir(SD, "/logs_2");
        listDir(SD, "/", 2);
        writeFile(SD, LOG_FILE_PATH, "Hello ");
        appendFile(SD, LOG_FILE_PATH, "World!\n");
        readFile(SD, LOG_FILE_PATH);
        renameFile(SD, LOG_FILE_PATH, "/foo.txt");
        readFile(SD, "/foo.txt");
        testFileIO(SD, LOG_FILE_PATH);
        
    }
};
/*
bool openWrite(const char * path, const char * message)
{
  ESP_LOGI(TAG,"Writing file: %s\r\n", path);
  logFile = SPIFFS.open(path, FILE_WRITE);
  if(!logFile){
      ESP_LOGE(TAG,"- failed to open file for writing");
      return false;
  }
  return true;
}

bool openAppend(const char * path, const char * message)
{
  ESP_LOGI(TAG,"Appending to file: %s\r\n", path);
  logFile = SPIFFS.open(path, FILE_APPEND);
  if(!logFile){
      ESP_LOGE(TAG,"- failed to open file for appending");
      return false;
  }
  return true;
}

void writeToFile(fs::File& f, const char * message)
{
  if(!f.print(message)){
    ESP_LOGE(TAG,"- write failed");
  }
  f.flush();
}

void readFile(const char * path){
  ESP_LOGI(TAG,"Reading file: %s\r\n", path);

  logFile = SPIFFS.open(path);
  if(!logFile || logFile.isDirectory()){
      ESP_LOGE(TAG,"- failed to open file for reading");
      return;
  }

  ESP_LOGI(TAG,"- read from file:");
  while(logFile.available()){
      Serial.write(logFile.read());
  }
  logFile.close();
}

void closeFile(fs::File& f)
{
  f.flush();
  f.close();
}
*/