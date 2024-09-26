#ifndef GROUND_IMAGE__GROUND_IMAGE_DISPLAY_HPP_
#define GROUND_IMAGE__GROUND_IMAGE_DISPLAY_HPP_

#include <QtCore>  // NOLINT

#include "rviz_ground_image/utils/static_image_texture.hpp"

#include "rviz_common/message_filter_display.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/tf_frame_property.hpp"
#include "rviz_common/render_panel.hpp"

#include "rviz_default_plugins/visibility_control.hpp"
#include "rviz_default_plugins/displays/image/ros_image_texture.hpp"
#include "rviz_rendering/render_window.hpp"
#include <OgreManualObject.h>

#include <rviz_common/display.hpp>
#include <OgreSceneNode.h>

namespace rviz_ground_image
{
    class GroundImageDisplay : public rviz_common::Display
{
    Q_OBJECT
    public:
        GroundImageDisplay();
        virtual ~GroundImageDisplay() {}

        void onInitialize() override;
        void update(float wall_dt, float ros_dt) override;

    private:
        Ogre::SceneNode* scene_node_;  // Узел сцены, на котором размещается изображение
        void updateImagePosition();    // Функция для обновления позиции изображения
};
        
}  // namespace ground_image


#endif  // GROUND_IMAGE__GROUND_IMAGE_DISPLAY_HPP_