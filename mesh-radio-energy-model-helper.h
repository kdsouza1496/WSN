
//////////////  mesh-radio-energy-model-helper.h  //////////////////


#ifndef MESH_RADIO_ENERGY_MODEL_HELPER_H
#define MESH_RADIO_ENERGY_MODEL_HELPER_H

#include "ns3/energy-model-helper.h"
#include "ns3/wifi-radio-energy-model.h"

namespace ns3 {

class MeshPointDevice;
/**
 * \ingroup energy
 * \brief Assign WifiRadioEnergyModel to wifi devices.
 *
 * This installer installs WifiRadioEnergyModel for only WifiNetDevice objects.
 *
 */
class MeshRadioEnergyModelHelper : public DeviceEnergyModelHelper
{
public:
  /**
   * Construct a helper which is used to add a radio energy model to a node
   */
  MeshRadioEnergyModelHelper ();

  /**
   * Destroy a RadioEnergy Helper
   */
  ~MeshRadioEnergyModelHelper ();

  /**
   * \param name the name of the attribute to set
   * \param v the value of the attribute
   *
   * Sets an attribute of the underlying PHY object.
   */
  void Set (std::string name, const AttributeValue &v);

  /**
   * \param callback Callback function for energy depletion handling.
   *
   * Sets the callback to be invoked when energy is depleted.
   */
  void SetDepletionCallback (
    WifiRadioEnergyModel::WifiRadioEnergyDepletionCallback callback);


private:
  /**
   * \param device Pointer to the NetDevice to install DeviceEnergyModel.
   *
   * Implements DeviceEnergyModel::Install.
   */
  virtual Ptr<DeviceEnergyModel> DoInstall (Ptr<NetDevice> device,
                                            Ptr<EnergySource> source) const;

private:
  ObjectFactory m_radioEnergy;
  WifiRadioEnergyModel::WifiRadioEnergyDepletionCallback m_depletionCallback;

};

} // namespace ns3

#endif /* WIFI_RADIO_ENERGY_MODEL_HELPER_H */
