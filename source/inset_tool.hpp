# ##### BEGIN GPL LICENSE BLOCK #####
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software Foundation,
#  Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ##### END GPL LICENSE BLOCK #####

#pragma once

#include "common.hpp"

#include <lxsdk/lx_channelui.hpp>
#include <lxsdk/lx_action.hpp>
#include <lxsdk/lx_layer.hpp>
#include <lxsdk/lx_pmodel.hpp>
#include <lxsdk/lx_plugin.hpp>
#include <lxsdk/lx_seltypes.hpp>
#include <lxsdk/lx_tool.hpp>
#include <lxsdk/lx_toolui.hpp>
#include <lxsdk/lx_vector.hpp>
#include <lxsdk/lx_vmodel.hpp>
#include <lxsdk/lxu_attributes.hpp>

#define SRVNAME_TOOLOP "toolop.smartInset"

class CInsetToolOp : public CLxImpl_ToolOperation
{
public:
    LxResult top_Evaluate(ILxUnknownID vts) LXx_OVERRIDE;

    CLxUser_Subject2Packet subject;
    unsigned               offset_subject = 0;
    LXtMarkMode            select_mode = 0;
    int                    mode = MODE_SAFE;
    double                 size = 0.15;
    double                 threshold = 0.01;
    bool                   merge = false;
    double                 merge_dist = 0.04;
    bool                   flat = false;
    double                 align = 0.0;
    bool                   connect = true;
};

class CInsetTool : public CLxImpl_Tool,
                   public CLxImpl_ToolModel,
                   public CLxDynamicAttributes,
                   public CLxImpl_ChannelUI
{
public:
    CInsetTool();

    void        tool_Reset() LXx_OVERRIDE;
    LXtObjectID tool_VectorType() LXx_OVERRIDE;
    const char* tool_Order() LXx_OVERRIDE;
    LXtID4      tool_Task() LXx_OVERRIDE;
    LxResult    tool_GetOp(void** ppvObj, unsigned flags) LXx_OVERRIDE;

    LxResult tmod_Enable(ILxUnknownID obj) LXx_OVERRIDE;

    using CLxDynamicAttributes::atrui_UIHints;
    void atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints) LXx_OVERRIDE;

    LxResult cui_Enabled(const char* channelName, ILxUnknownID msg, ILxUnknownID item, ILxUnknownID read) LXx_OVERRIDE;
    LxResult cui_DependencyCount(const char* channelName, unsigned* count) LXx_OVERRIDE;
    LxResult cui_DependencyByIndex(const char* channelName, unsigned index, LXtItemType* depItemType, const char** depChannel) LXx_OVERRIDE;

    bool TestPolygon();

    static void initialize();
    static LXtTagInfoDesc descInfo[];

private:
    CLxUser_LayerService  layer_service;
    CLxUser_MeshService   mesh_service;
    CLxUser_VectorType    vector_type;
    LXtMarkMode           select_mode = 0;
    unsigned              offset_subject = 0;
    LXtItemType           item_type = 0;
};
