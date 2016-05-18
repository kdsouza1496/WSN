
#ifndef ENERGY_MODEL_H
#define ENERGY_MODEL_H

#include "ns3/object.h"
#include "ns3/ptr.h"
#include "ns3/nstime.h"

namespace ns3 {

class EnergyModel : public Object
{
public:
  static const InterfaceId iid;

  EnergyModel (
    double ampHours,
    double volts,
    double idlePower,
    double receivePower,
    double transmitPower);

  virtual ~EnergyModel ();

  double GetCapacity (Time t);

  double GetTotalIdlePower (Time t);
  double GetTotalTransmitPower (void);
  double GetTotalReceivePower (void);

  bool DrawTransmitPower (Time t);
  bool DrawReceivePower (Time t);

private:
  double m_capacity;
  double m_idlePower;
  double m_receivePower;
  double m_transmitPower;
  double m_totalTransmitPower;
  double m_totalReceivePower;

};
}; // namespace ns3

#endif /* ENERGY_MODEL_H */ 