# idf_component_register(SRCS "main.cpp"
#                     INCLUDE_DIRS "." 
                    
                  
#                     )

#                     set(LIBRARY_SRCS, "libraries/Adafruit-GFX-Library/Adafruit_GFX.cpp" "libraries/Adafruit_BusIO/Adafruit_I2CDevice.cpp" "libraries/Adafruit_SSD1306/Adafruit_SSD1306.cpp" )


#                     set(includedirs, "libraries/Adafruit-GFX-Library" "libraries/Adafruit_BusIO" "libraries/Adafruit_SSD1306" )

idf_component_register(
   SRCS "main.cpp" 
   "libraries/Adafruit-GFX-Library/Adafruit_GFX.cpp"
   "libraries/Adafruit_BusIO/Adafruit_I2CDevice.cpp"
   "libraries/Adafruit_SSD1306/Adafruit_SSD1306.cpp"
   SRC_DIRS               "." 
                          "libraries/Adafruit_SSD1306" "libraries/Adafruit-GFX-Library/Fonts" "libraries/Adafruit-GFX-Library" "libraries/Adafruit_BusIO"  
   INCLUDE_DIRS           "."
                          "libraries/Adafruit_SSD1306" "libraries/Adafruit-GFX-Library/Fonts" "libraries/Adafruit-GFX-Library" "libraries/Adafruit_BusIO"  
)