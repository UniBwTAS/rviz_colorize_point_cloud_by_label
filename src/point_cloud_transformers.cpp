#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/properties/editable_enum_property.h>

#include <rviz/ogre_helpers/color_material_helper.h>

#include "point_cloud_transformers.h"

namespace rviz
{

uint8_t LabelPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    updateChannels(cloud);
    return Support_Color;
}

bool LabelPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
                                   uint32_t mask,
                                   const Ogre::Matrix4& transform,
                                   V_PointCloudPoint& points_out)
{
    if (!(mask & Support_Color))
    {
        return false;
    }

    int32_t index = findChannelIndex(cloud, channel_name_property_->getStdString());

    if (index == -1)
    {
        return false;
    }

    const uint32_t offset = cloud->fields[index].offset;
    const uint8_t type = cloud->fields[index].datatype;
    const uint32_t point_step = cloud->point_step;
    const uint32_t num_points = cloud->width * cloud->height;

    for (uint32_t i = 0; i < num_points; ++i)
    {
        auto val = valueFromCloud<uint16_t>(cloud, offset, type, point_step, i);
        points_out[i].color =
            ColorHelper::getOgreColorFromList(val % static_cast<int>(ColorHelper::getColorListSize()));
    }

    return true;
}

uint8_t LabelPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    return 255;
}

void LabelPCTransformer::createProperties(Property* parent_property, uint32_t mask, QList<Property*>& out_props)
{
    if (mask & Support_Color)
    {
        channel_name_property_ = new EditableEnumProperty("Channel Name",
                                                          "sem_label",
                                                          "Select the channel to use to colorize by label",
                                                          parent_property,
                                                          SIGNAL(needRetransform()),
                                                          this);

        out_props.push_back(channel_name_property_);
    }
}

void LabelPCTransformer::updateChannels(const sensor_msgs::PointCloud2ConstPtr& cloud)
{
    std::vector<std::string> channels;
    for (const auto& field : cloud->fields)
    {
        channels.push_back(field.name);
    }
    std::sort(channels.begin(), channels.end());

    if (channels != available_channels_)
    {
        channel_name_property_->clearOptions();
        for (auto & channel : channels)
        {
            if (channel.empty())
            {
                continue;
            }
            channel_name_property_->addOptionStd(channel);
        }
        available_channels_ = channels;
    }
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::LabelPCTransformer, rviz::PointCloudTransformer)
