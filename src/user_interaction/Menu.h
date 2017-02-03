#ifndef MICROMOUSE_MENU_H
#define MICROMOUSE_MENU_H

// usage: menuFunction { <function-body> }
#define menuFunction [] () -> void

class MenuItem
{
  public:
    typedef void (*Function)();

    MenuItem(const char *name, Function function);

    // constructs an array-terminating "null" item
    MenuItem();

    bool isNull() const;

    const char *name() const;
    Function function() const;

  private:
    const char *name_;
    Function function_;
};

class Menu
{
  public:
    // items array must be terminated with a "null" item
    Menu(const MenuItem items[], bool append_back_item = true);

    // returns false if the appended back item was selected, else true
    bool run();

  private:
    static const size_t kMaxLength = 10 + 1;
    static const size_t kNameMaxLength = sizeof("0000") / sizeof(char);

    const size_t length_;

    MenuItem items_[kMaxLength];
    const char *names_[kMaxLength];

    bool show_back_item_;

    size_t countLength(const MenuItem array[]);
};

#endif
