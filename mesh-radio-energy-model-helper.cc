
//////////  mesh-radio-energy-model-helper.cc  ///////////////

#include "ns3/mesh-radio-energy-model-helper.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/wifi-phy.h"
#include "ns3/wifi-net-device.h"
#include "ns3/mesh-point-device.h"
#include "ns3/config.h"
#include "ns3/names.h"

namespace ns3 {

MeshRadioEnergyModelHelper::MeshRadioEnergyModelHelper ()
{
  m_radioEnergy.SetTypeId ("ns3::WifiRadioEnergyModel");
  m_depletionCallback.Nullify ();
}

MeshRadioEnergyModelHelper::~MeshRadioEnergyModelHelper ()
{
}

void
MeshRadioEnergyModelHelper::Set (std::string name, const AttributeValue &v)
{
  m_radioEnergy.Set (name, v);
}

void
MeshRadioEnergyModelHelper::SetDepletionCallback (
  WifiRadioEnergyModel::WifiRadioEnergyDepletionCallback callback)
{
  m_depletionCallback = callback;
}

/*
 * Private function starts here.
 */

Ptr<DeviceEnergyModel>
MeshRadioEnergyModelHelper::DoInstall (Ptr<NetDevice> device,
                                       Ptr<EnergySource> source) const
{
  NS_ASSERT (device != NULL);
  NS_ASSERT (source != NULL);
  // check if device is MeshPointDevice
  std::string deviceName = device->GetInstanceTypeId ().GetName ();
  if (deviceName.compare ("ns3::MeshPointDevice") != 0)
    {
      NS_FATAL_ERROR ("NetDevice type is not MeshPointDevice!");
    }
  // Ptr<Node> node = device->GetNode ();
 
  Ptr<WifiRadioEnergyModel> model = m_radioEnergy.Create ()->GetObject<WifiRadioEnergyModel> ();
  NS_ASSERT (model != NULL);
  // set energy source pointer
  model->SetEnergySource (source);
  // set energy depletion callback
  model->SetEnergyDepletionCallback (m_depletionCallback);
  // add model to device model list in energy source
  source->AppendDeviceEnergyModel (model);
  // create and register energy model phy listener
 

//////////   START EDIT  ////////////
  Ptr<MeshPointDevice> mp = DynamicCast<MeshPointDevice> (device);
  std::vector<Ptr<NetDevice> > interfaces = mp->GetInterfaces ();
 
  for (std::vector<Ptr<NetDevice> >::const_iterator i = interfaces.begin (); i != interfaces.end (); i++)
  {
    Ptr<WifiNetDevice> wifiDevice = (*i)->GetObject<WifiNetDevice> ();
    Ptr<WifiPhy> wifiPhy = wifiDevice->GetPhy ();
    wifiPhy->RegisterListener (model->GetPhyListener ());
  }
//////////   END EDIT   ////////////
