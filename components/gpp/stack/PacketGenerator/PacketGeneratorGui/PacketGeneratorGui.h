#ifndef PACKETGENERATORGUI_H
#define PACKETGENERATORGUI_H

#include <string>

class PacketGeneratorGuiWrapper;

class PacketGeneratorGui
{
public:
  PacketGeneratorGui();
  ~PacketGeneratorGui();
  void waitForPacket(int& length);

private:
  PacketGeneratorGuiWrapper* plot_;
};

#endif // PACKETGENERATORGUI_H
