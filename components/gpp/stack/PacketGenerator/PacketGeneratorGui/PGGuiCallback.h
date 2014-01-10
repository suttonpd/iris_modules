#ifndef PGGUICALLBACK_H
#define PGGUICALLBACK_H

class PGGuiCallback
{
public:
  virtual void generatePacket(int length) = 0;
};

#endif // PGGUICALLBACK_H
