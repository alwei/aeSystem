CXX = g++
CXXFLAGS = -Wall -g -O0
TARGET = game.exe

SRC = main.cpp
OBJS = $(patsubst %.cpp,%.o,$(SRC))
DEPENDS = $(patsubst %.cpp,%.d,$(SRC))

$(TARGET): $(OBJS)
	$(CXX) -o $@ $(OBJS)

.cpp.o:
	$(CXX) $(CXXFLAGS) -c $<

.PHONY: clean depend
clean:
	-$(RM) $(OBJS) $(DEPENDS) $(TARGET)

%.d: %.cpp
	@set -e; $(CXX) -MM $(CXXFLAGS) $< \
		| sed 's/\($*\)\.o[ :]*/\1.o $@ : /g' > $@; \
		[ -s $@ ] || rm -f $@
	-include $(DEPENDS)

