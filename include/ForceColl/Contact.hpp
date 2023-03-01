namespace ForceColl
{
template<template<class, class> class MapType, class PatchID>
MapType<PatchID, sva::ForceVecd> calcWrenchList(const MapType<PatchID, std::shared_ptr<Contact>> & contactList,
                                                const Eigen::VectorXd & wrenchRatio,
                                                const Eigen::Vector3d & momentOrigin)
{
  MapType<PatchID, sva::ForceVecd> wrenchList;
  int wrenchRatioIdx = 0;
  for(const auto & contactKV : contactList)
  {
    wrenchList.emplace(contactKV.first,
                       contactKV.second->calcWrench(
                           wrenchRatio.segment(wrenchRatioIdx, contactKV.second->graspMat_.cols()), momentOrigin));
    wrenchRatioIdx += static_cast<int>(contactKV.second->graspMat_.cols());
  }
  return wrenchList;
}
} // namespace ForceColl
