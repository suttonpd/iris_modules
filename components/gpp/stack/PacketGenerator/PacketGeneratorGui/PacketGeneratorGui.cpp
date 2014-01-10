#include "PacketGeneratorGui.h"
#include "PacketGeneratorGuiWrapper.h"

using namespace std;

PacketGeneratorGui::PacketGeneratorGui()
{
  plot_ = new PacketGeneratorGuiWrapper;
}

PacketGeneratorGui::~PacketGeneratorGui()
{
  if(plot_)
    delete plot_;
}

void PacketGeneratorGui::waitForPacket(int& length)
{
  plot_->waitForPacket(length);
}
