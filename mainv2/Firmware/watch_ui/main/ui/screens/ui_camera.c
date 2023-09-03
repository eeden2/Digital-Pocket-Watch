// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.1
// LVGL version: 8.3.6
// Project name: Smartwatch

#include "../ui.h"

void ui_camera_screen_init(void)
{
    ui_camera = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_camera, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_bg_3 = lv_img_create(ui_camera);
    lv_img_set_src(ui_bg_3, &ui_img_bg3_png);
    lv_obj_set_width(ui_bg_3, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_bg_3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_bg_3, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_bg_3, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
    lv_obj_clear_flag(ui_bg_3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_avatar_label = lv_label_create(ui_camera);
    lv_obj_set_width(ui_avatar_label, LV_SIZE_CONTENT);   /// 5
    lv_obj_set_height(ui_avatar_label, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_avatar_label, 4);
    lv_obj_set_y(ui_avatar_label, -39);
    lv_obj_set_align(ui_avatar_label, LV_ALIGN_CENTER);
    lv_label_set_text(ui_avatar_label, "Connecting");
    lv_obj_set_style_text_color(ui_avatar_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_avatar_label, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_avatar_label, &ui_font_H1, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_dot1 = lv_obj_create(ui_camera);
    lv_obj_set_width(ui_dot1, 10);
    lv_obj_set_height(ui_dot1, 10);
    lv_obj_set_x(ui_dot1, 105);
    lv_obj_set_y(ui_dot1, 12);
    lv_obj_set_align(ui_dot1, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_dot1, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_dot1, 10, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_dot2 = lv_obj_create(ui_camera);
    lv_obj_set_width(ui_dot2, 6);
    lv_obj_set_height(ui_dot2, 6);
    lv_obj_set_x(ui_dot2, 105);
    lv_obj_set_y(ui_dot2, -6);
    lv_obj_set_align(ui_dot2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_dot2, LV_OBJ_FLAG_PRESS_LOCK | LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_radius(ui_dot2, 10, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_dot2, lv_color_hex(0x676767), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_dot2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_add_event_cb(ui_bg_3, ui_event_bg_3, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_dot2, ui_event_dot2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_camera, ui_event_camera, LV_EVENT_ALL, NULL);

}
