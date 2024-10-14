#include <rviz/default_plugin/point_cloud_transformers.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/editable_enum_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/color_property.h>

#include <ogre_helpers/color_material_helper.h>

#include "point_cloud_transformers.h"

namespace rviz
{
    static void getRainbowColorLabel(float value, Ogre::ColourValue& color)
    {
        // this is HSV color palette with hue values going only from 0.0 to 0.833333.

        value = std::min(value, 1.0f);
        value = std::max(value, 0.0f);

        float h = value * 5.0f + 1.0f;
        int i = floor(h);
        float f = h - i;
        if (!(i & 1))
            f = 1 - f; // if i is even
        float n = 1 - f;

        if (i <= 1)
            color[0] = n, color[1] = 0, color[2] = 1;
        else if (i == 2)
            color[0] = 0, color[1] = n, color[2] = 1;
        else if (i == 3)
            color[0] = 0, color[1] = 1, color[2] = n;
        else if (i == 4)
            color[0] = n, color[1] = 1, color[2] = 0;
        else if (i >= 5)
            color[0] = 1, color[1] = n, color[2] = 0;
    }

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



    uint8_t IntensityLabelPCTransformer::supports(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        updateChannels(cloud);
        return Support_Color;
    }

    bool IntensityLabelPCTransformer::transform(const sensor_msgs::PointCloud2ConstPtr& cloud,
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
            if (channel_name_property_->getStdString() == "intensity")
            {
                index = findChannelIndex(cloud, "intensities");
                if (index == -1)
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }

        bool show_only_activated = show_only_property_->getBool();
        float show_only_desired_value = show_only_value_property_->getFloat();
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

        float min_intensity = 999999.0f;
        float max_intensity = -999999.0f;
        if (auto_compute_intensity_bounds_property_->getBool())
        {
            for (uint32_t i = 0; i < num_points; ++i)
            {
                float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
                if (!show_only_activated)
                {
                    min_intensity = std::min(val, min_intensity);
                    max_intensity = std::max(val, max_intensity);
                }
                else
                {
                    const uint32_t show_only_offset = cloud->fields[show_only_index].offset;
                    const uint8_t show_only_type = cloud->fields[show_only_index].datatype;
                    auto show_only_val = valueFromCloud<uint16_t>(cloud, show_only_offset, show_only_type, point_step, i);
                    if(show_only_val == show_only_desired_value)
                    {
                        min_intensity = std::min(val, min_intensity);
                        max_intensity = std::max(val, max_intensity);
                    }
                }
            }

            min_intensity = std::max(-999999.0f, min_intensity);
            max_intensity = std::min(999999.0f, max_intensity);
            min_intensity_property_->setFloat(min_intensity);
            max_intensity_property_->setFloat(max_intensity);
        }
        else
        {
            min_intensity = min_intensity_property_->getFloat();
            max_intensity = max_intensity_property_->getFloat();
        }

        float diff_intensity = max_intensity - min_intensity;
        if (diff_intensity == 0)
        {
            // If min and max are equal, set the diff to something huge so
            // when we divide by it, we effectively get zero.  That way the
            // point cloud coloring will be predictably uniform when min and
            // max are equal.
            diff_intensity = 1e20;
        }
        Ogre::ColourValue max_color = max_color_property_->getOgreColor();
        Ogre::ColourValue min_color = min_color_property_->getOgreColor();

        if (use_rainbow_property_->getBool())
        {
            for (uint32_t i = 0; i < num_points; ++i)
            {
                float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
                float value = 1.0 - (val - min_intensity) / diff_intensity;
                if (invert_rainbow_property_->getBool())
                {
                    value = 1.0 - value;
                }
                getRainbowColorLabel(value, points_out[i].color);

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
        }
        else
        {
            for (uint32_t i = 0; i < num_points; ++i)
            {
                float val = valueFromCloud<float>(cloud, offset, type, point_step, i);
                float normalized_intensity = (val - min_intensity) / diff_intensity;
                normalized_intensity = std::min(1.0f, std::max(0.0f, normalized_intensity));
                points_out[i].color.r =
                        max_color.r * normalized_intensity + min_color.r * (1.0f - normalized_intensity);
                points_out[i].color.g =
                        max_color.g * normalized_intensity + min_color.g * (1.0f - normalized_intensity);
                points_out[i].color.b =
                        max_color.b * normalized_intensity + min_color.b * (1.0f - normalized_intensity);

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
        }

        return true;
    }

    uint8_t IntensityLabelPCTransformer::score(const sensor_msgs::PointCloud2ConstPtr& cloud)
    {
        return 255;
    }

    void IntensityLabelPCTransformer::createProperties(Property* parent_property, uint32_t mask, QList<Property*>& out_props)
    {

        if (mask & Support_Color)
        {

            channel_name_property_ =
                    new EditableEnumProperty("Channel Name", "intensity",
                                             "Select the channel to use to compute the intensity", parent_property,
                                             SIGNAL(needRetransform()), this);

            use_rainbow_property_ =
                    new BoolProperty("Use rainbow", true,
                                     "Whether to use a rainbow of colors or interpolate between two",
                                     parent_property, &IntensityLabelPCTransformer::updateUseRainbow, this);
            invert_rainbow_property_ =
                    new BoolProperty("Invert Rainbow", false, "Whether to invert rainbow colors", parent_property,
                                     &IntensityLabelPCTransformer::updateUseRainbow, this);

            min_color_property_ =
                    new ColorProperty("Min Color", Qt::black,
                                      "Color to assign the points with the minimum intensity.  "
                                      "Actual color is interpolated between this and Max Color.",
                                      parent_property,
                                      SIGNAL(needRetransform()), this);

            max_color_property_ =
                    new ColorProperty("Max Color", Qt::white,
                                      "Color to assign the points with the maximum intensity.  "
                                      "Actual color is interpolated between this and Min Color.",
                                      parent_property,
                                      SIGNAL(needRetransform()), this);

            auto_compute_intensity_bounds_property_ =
                    new BoolProperty("Autocompute Intensity Bounds", true,
                                     "Whether to automatically compute the intensity min/max values.",
                                     parent_property, &IntensityLabelPCTransformer::updateAutoComputeIntensityBounds,
                                     this);

            min_intensity_property_ = new FloatProperty(
                    "Min Intensity", 0,
                    "Minimum possible intensity value, used to interpolate from Min Color to Max Color for a point.",
                    parent_property);

            max_intensity_property_ = new FloatProperty(
                    "Max Intensity", 4096,
                    "Maximum possible intensity value, used to interpolate from Min Color to Max Color for a point.",
                    parent_property);


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
                    new FloatProperty("Equal To", 0, "Select the value", show_only_property_, SIGNAL(needRetransform()), this);


            out_props.push_back(channel_name_property_);
            out_props.push_back(use_rainbow_property_);
            out_props.push_back(invert_rainbow_property_);
            out_props.push_back(min_color_property_);
            out_props.push_back(max_color_property_);
            out_props.push_back(auto_compute_intensity_bounds_property_);
            out_props.push_back(min_intensity_property_);
            out_props.push_back(max_intensity_property_);
            out_props.push_back(show_only_property_);

                updateUseRainbow();
                updateAutoComputeIntensityBounds();




        }
    }

    void IntensityLabelPCTransformer::updateChannels(const sensor_msgs::PointCloud2ConstPtr& cloud)
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

    void IntensityLabelPCTransformer::updateAutoComputeIntensityBounds()
    {
        bool auto_compute = auto_compute_intensity_bounds_property_->getBool();
        min_intensity_property_->setReadOnly(auto_compute);
        max_intensity_property_->setReadOnly(auto_compute);
        if (auto_compute)
        {
            disconnect(min_intensity_property_, &Property::changed, this,
                       &IntensityLabelPCTransformer::needRetransform);
            disconnect(max_intensity_property_, &Property::changed, this,
                       &IntensityLabelPCTransformer::needRetransform);
        }
        else
        {
            connect(min_intensity_property_, &Property::changed, this,
                    &IntensityLabelPCTransformer::needRetransform);
            connect(max_intensity_property_, &Property::changed, this,
                    &IntensityLabelPCTransformer::needRetransform);
        }
        Q_EMIT needRetransform();
    }

    void IntensityLabelPCTransformer::updateUseRainbow()
    {
        bool use_rainbow = use_rainbow_property_->getBool();
        invert_rainbow_property_->setHidden(!use_rainbow);
        min_color_property_->setHidden(use_rainbow);
        max_color_property_->setHidden(use_rainbow);
        Q_EMIT needRetransform();
    }


} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::LabelPCTransformer, rviz::PointCloudTransformer)
PLUGINLIB_EXPORT_CLASS(rviz::IntensityLabelPCTransformer, rviz::PointCloudTransformer)
