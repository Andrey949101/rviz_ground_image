#include <OgreMaterialManager.h>
#include <OgreTextureManager.h>  // Для работы с текстурами
#include "rviz_ground_image/ground_image_display.hpp"
#include "rviz_ground_image/static_ground_image.hpp"

namespace rviz_ground_image
{

StaticGroundImage::StaticGroundImage(rviz_common::DisplayContext * context)
  : rviz_common::Display()
{
    this->context_ = context;
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "StaticGroundImage constructor called");
    setupProperties();
}

StaticGroundImage::StaticGroundImage()
  : rviz_common::Display()
{
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "StaticGroundImage constructor called");
    setupProperties();
}

void StaticGroundImage::onInitialize()
{
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "onInitialize called");

    // Инициализация сцены и объектов для изображения
    object_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Scene node created");

    screen_rect_ = scene_manager_->createManualObject();
    object_node_->attachObject(screen_rect_);
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Manual object created and attached to node");

    // Создаем текстуру и материал для изображения
    texture_ = std::make_unique<StaticImageTexture>();
    material_ = Ogre::MaterialManager::getSingleton().create(
        "StaticGroundImageMaterial", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Material and texture created");

    // Настройка размеров и изображения при инициализации
    updateImageAndDimensions();
}

void StaticGroundImage::update(float /* dt */, float /* ros_dt */)
{
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Update called");

    // Автоматически обновляем размеры изображения, если оно загружено
    if (image_loaded_)
    {
        RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Image is loaded. Updating dimensions...");
        updateImageAndDimensions();
    }
}

void StaticGroundImage::reset()
{
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Reset called");

    Display::reset();
    if (screen_rect_)
    {
        screen_rect_->clear();
    }
}

void StaticGroundImage::updateImageAndDimensions()
{
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "updateImageAndDimensions called");

    if (!texture_)
    {
        RCLCPP_WARN(rclcpp::get_logger("rviz_ground_image"), "Texture is not initialized!");
        return;
    }

    std::string image_path = image_path_property_->getStdString();
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Trying to load image: %s", image_path.c_str());

    if (!image_path.empty())
    {
        // Используем OGRE для загрузки текстуры
        Ogre::TexturePtr texture = Ogre::TextureManager::getSingleton().load(
            image_path, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

        // Проверка валидности текстуры через оператор bool
        if (texture)
        {
            RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Image loaded successfully: %s", image_path.c_str());
            image_loaded_ = true;
            setupScreenRectangle();
        }
        else
        {
            RCLCPP_WARN(rclcpp::get_logger("rviz_ground_image"), "Failed to load image: %s", image_path.c_str());
        }
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("rviz_ground_image"), "Image path is empty.");
    }
}

void StaticGroundImage::setupProperties()
{
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Setting up properties");

    // Установка пути к изображению по умолчанию
    image_path_property_ = new rviz_common::properties::StringProperty("Image Path", "/home/user/ros2_ws/src/autoware_launch/autoware_launch/rviz/image/logo_.png",
        "Path to the image file to display.",
        this, SLOT(updateImageAndDimensions()));

    // Параметры кадра, ширины и высоты
    frame_property_ = new rviz_common::properties::TfFrameProperty("Frame", "map",
        "Reference frame for the image.",
        this, nullptr, this);

    width_property_ = new rviz_common::properties::FloatProperty("Width", 10.0,
        "Width of the image in meters.",
        this, SLOT(updateImageAndDimensions()));

    height_property_ = new rviz_common::properties::FloatProperty("Height", 10.0,
        "Height of the image in meters.",
        this, SLOT(updateImageAndDimensions()));

    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Properties setup completed");
}

void StaticGroundImage::setupScreenRectangle()
{
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Setting up screen rectangle");

    if (!screen_rect_)
    {
        screen_rect_ = scene_manager_->createManualObject();
        object_node_->attachObject(screen_rect_);
        RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Manual object created for screen rectangle");
    }

    screen_rect_->clear();

    // Получаем размеры изображения из свойств
    float width = width_property_->getFloat();
    float height = height_property_->getFloat();
    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Image dimensions: width = %f, height = %f", width, height);

    // Устанавливаем координаты для отображения изображения
    screen_rect_->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_STRIP);
    screen_rect_->position(-width / 2, height / 2, 0.0);
    screen_rect_->textureCoord(0.0, 0.0);

    screen_rect_->position(width / 2, height / 2, 0.0);
    screen_rect_->textureCoord(1.0, 0.0);

    screen_rect_->position(-width / 2, -height / 2, 0.0);
    screen_rect_->textureCoord(0.0, 1.0);

    screen_rect_->position(width / 2, -height / 2, 0.0);
    screen_rect_->textureCoord(1.0, 1.0);

    screen_rect_->end();

    object_node_->setPosition(0, 0, 0);
    object_node_->setScale(width, height, 1.0);

    RCLCPP_INFO(rclcpp::get_logger("rviz_ground_image"), "Screen rectangle setup completed");
}

} // namespace rviz_ground_image