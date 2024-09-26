#include <OgreMaterialManager.h>
#include <OgreTechnique.h>
#include <OgrePass.h>
#include <OgreEntity.h>
#include <OgreCamera.h>
#include <OgrePlane.h>
#include <OgreMeshManager.h>
#include <OgreTextureManager.h>
#include <rviz_common/display_context.hpp>
#include <rviz_common/view_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rviz_ground_image/ground_image_display.hpp"
#include "rviz_ground_image/static_ground_image.hpp"

namespace rviz_ground_image
{

GroundImageDisplay::GroundImageDisplay()
{
    // Конструктор, инициализация
}

void GroundImageDisplay::onInitialize()
{
    // Инициализация изображения
    scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    
    // Загружаем текстуру и создаем материал для изображения
    Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().load(
        "ground_image.png", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    
    Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create(
        "GroundImageMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    
    material->getTechnique(0)->getPass(0)->createTextureUnitState(texture->getName());
    material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
    material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
    
    // Создаем плоскость (Plane) для размещения изображения
    Ogre::Plane plane(Ogre::Vector3::UNIT_Z, 0);
    Ogre::MeshManager::getSingleton().createPlane(
        "GroundImagePlane",
        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
        plane, 
        1000, 1000, // Размеры плоскости (измените по необходимости)
        10, 10, // Разбиение
        true, 
        1, 
        5, 5, 
        Ogre::Vector3::UNIT_Y
    );

    // Создаем объект Entity на основе созданной плоскости
    Ogre::Entity* ground_entity = scene_manager_->createEntity("GroundImagePlane");
    ground_entity->setMaterialName("GroundImageMaterial");
    scene_node_->attachObject(ground_entity);
}

void GroundImageDisplay::updateImagePosition()
{
    // Получаем текущий контроллер вида через ViewManager
    rviz_common::ViewManager* view_manager = context_->getViewManager();
    if (view_manager)
    {
        auto* view_controller = view_manager->getCurrent();
        if (view_controller)
        {
            Ogre::Camera* camera = view_controller->getCamera();
            if (camera)
            {
                Ogre::Vector3 camera_position = camera->getPosition();

                // Используем позицию камеры для центрирования изображения
                Ogre::Vector3 image_position(camera_position.x, camera_position.y, 0.0);
                scene_node_->setPosition(image_position);

                RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Updated image position to X: %f, Y: %f", camera_position.x, camera_position.y);
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("rviz_ground_image"), "Camera not found in view controller.");
            }
        }
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("rviz_ground_image"), "ViewManager not found.");
    }
}

void GroundImageDisplay::update(float wall_dt, float ros_dt)
{
    // Обновляем положение изображения при каждом кадре
    updateImagePosition();
}

}  // namespace rviz_ground_image

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_ground_image::GroundImageDisplay, rviz_common::Display)
