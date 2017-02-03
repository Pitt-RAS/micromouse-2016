#include <Arduino.h>
#include "../device/PersistantStorage.h"
#include "UserInterface.h"
#include "../driver.h"
#include "Menu.h"

static void doNothing() {};

MenuItem::MenuItem(const char *name, Function function) :
  name_(name), function_(function)
{}

MenuItem::MenuItem() : MenuItem(nullptr, nullptr)
{}

bool MenuItem::isNull() const
{
  return name_ == nullptr || function_ == nullptr;
}

const char *MenuItem::name() const
{
  return name_;
}

MenuItem::Function MenuItem::function() const
{
  return function_;
}

Menu::Menu(const MenuItem items[], bool append_back_item) :
  length_(countLength(items) + 1), show_back_item_(append_back_item)
{
  for (size_t i = 0; i < length_ - 1; i++)
    items_[i] = items[i];

  items_[length_ - 1] = MenuItem("BACK", doNothing);

  for (size_t i = 0; i < length_; i++)
    names_[i] = items_[i].name();
}

bool Menu::run()
{
  size_t length = show_back_item_ ? length_ : length_ - 1;
  size_t name_length = kNameMaxLength - 1;

  size_t selected = gUserInterface.getString(names_, length, name_length);
  items_[selected].function()();

  return selected != length_ - 1;
}

size_t Menu::countLength(const MenuItem array[])
{
  size_t i;
  for (i = 0; i < kMaxLength - 1 && !array[i].isNull(); i++);
  return i;
}
