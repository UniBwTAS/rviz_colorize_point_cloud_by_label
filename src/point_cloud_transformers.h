#pragma once

#include <rviz/default_plugin/point_cloud_transformer.h>

namespace rviz
{

class EditableEnumProperty;
class IntProperty;
class BoolProperty;
class ColorProperty;
class FloatProperty;

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


class IntensityLabelPCTransformer : public PointCloudTransformer
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

    private Q_SLOTS:
        void updateUseRainbow();
        void updateAutoComputeIntensityBounds();

    private:
        std::vector<std::string> available_channels_;
        EditableEnumProperty* channel_name_property_;
        BoolProperty* show_only_property_;
        FloatProperty* show_only_value_property_;
        EditableEnumProperty* show_only_channel_name_property_;

        ColorProperty* min_color_property_;
        ColorProperty* max_color_property_;
        BoolProperty* auto_compute_intensity_bounds_property_;
        BoolProperty* use_rainbow_property_;
        BoolProperty* invert_rainbow_property_;
        FloatProperty* min_intensity_property_;
        FloatProperty* max_intensity_property_;

};
}; // namespace rviz
