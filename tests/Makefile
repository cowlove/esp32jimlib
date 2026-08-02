CXX ?= g++
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Werror
CRC_DIR ?= $(HOME)/Arduino/libraries/Arduino_CRC32/src

.PHONY: test clean

test: reliable_stream_framing_test
	./reliable_stream_framing_test

reliable_stream_framing_test: reliable_stream_framing_test.cpp \
		../src/reliableStreamFraming.h $(CRC_DIR)/Arduino_CRC32.cpp \
		$(CRC_DIR)/crc.cpp
	$(CXX) $(CXXFLAGS) -I../src -I$(CRC_DIR) reliable_stream_framing_test.cpp \
		$(CRC_DIR)/Arduino_CRC32.cpp $(CRC_DIR)/crc.cpp -o $@

clean:
	rm -f reliable_stream_framing_test
