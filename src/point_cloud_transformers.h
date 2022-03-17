#pragma once

#include <rviz/default_plugin/point_cloud_transformer.h>

namespace rviz
{

class EditableEnumProperty;
class IntProperty;
class BoolProperty;

class LabelPCTransformer : public PointCloudTransformer
{
    Q_OBJECT
  public:
    uint8_t supports(const sensor_msgs::PointCloud2ConstPtr& cloud) override;
    bool transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                   uint32_t mask,
                   const Ogre::Matrix4& transform,
                   V_PointCloudPoint& points_out) override;
    uint8_t score(const sensor_msgs::PointCloud2ConstPtr& cloud) override;
    void createProperties(Property* parent_property, uint32_t mask, QList<Property*>& out_props) override;
    void updateChannels(const sensor_msgs::PointCloud2ConstPtr& cloud);

  private:
    std::vector<std::string> available_channels_;
    EditableEnumProperty* channel_name_property_;
    BoolProperty* show_only_property_;
    IntProperty* show_only_value_property_;
    EditableEnumProperty* show_only_channel_name_property_;
};
}; // namespace rviz
