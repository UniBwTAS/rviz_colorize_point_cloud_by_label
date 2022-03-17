#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/int_property.h>

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

    bool show_only_activated = show_only_property_->getBool();
    uint16_t show_only_desired_value = show_only_value_property_->getInt();
    uint32_t show_only_index;
    if (show_only_activated)
    {
        show_only_index = findChannelIndex(cloud, show_only_channel_name_property_->getStdString());

        if (show_only_index == -1)
        {
            return false;
        }
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

        if (show_only_activated)
        {
            const uint32_t show_only_offset = cloud->fields[show_only_index].offset;
            const uint8_t show_only_type = cloud->fields[show_only_index].datatype;
            auto show_only_val = valueFromCloud<uint16_t>(cloud, show_only_offset, show_only_type, point_step, i);
            if(show_only_val != show_only_desired_value)
            {
                points_out[i].color.a = 0.f;
                // put those points to origin in order to not accidentally select them with the selection tool
                points_out[i].position.x = 0.f;
                points_out[i].position.y = 0.f;
                points_out[i].position.z = 0.f;
            }
        }
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
        show_only_property_ = new BoolProperty(
            "Show only", false, "Show only points with value", parent_property, SIGNAL(needRetransform()), this);
        show_only_property_->setDisableChildrenIfFalse(true);
        show_only_channel_name_property_ = new EditableEnumProperty("Channel Name",
                                                                    "sem_label",
                                                                    "Select the channel by which to hide",
                                                                    show_only_property_,
                                                                    SIGNAL(needRetransform()),
                                                                    this);
        show_only_value_property_ =
            new IntProperty("Equal To", 0, "Select the value", show_only_property_, SIGNAL(needRetransform()), this);

        out_props.push_back(channel_name_property_);
        out_props.push_back(show_only_property_);
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
        show_only_channel_name_property_->clearOptions();
        for (auto& channel : channels)
        {
            if (channel.empty())
            {
                continue;
            }
            channel_name_property_->addOptionStd(channel);
            show_only_channel_name_property_->addOptionStd(channel);
        }
        available_channels_ = channels;
    }
}

} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::LabelPCTransformer, rviz::PointCloudTransformer)
