/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2026 by Yoshiaki Tazaki.
 * All rights reserved.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

#include "inset_tool.hpp"

#include "padding_inset.hpp"
#include "safe_inset.hpp"

using namespace lx_err;

CInsetTool::CInsetTool()
{
    static const LXtTextValueHint inset_modes[] = {
        { MODE_SAFE, "safe" },
        { MODE_PADDING, "padding" },
        { 0, "=inset_mode" },
        0
    };

    dyna_Add(ATTRs_MODE, LXsTYPE_INTEGER);
    dyna_SetHint(ATTRi_MODE, inset_modes);
    dyna_Add(ATTRs_SIZE, LXsTYPE_DISTANCE);
    dyna_Add(ATTRs_THRESHOLD, LXsTYPE_FLOAT);
    dyna_Add(ATTRs_MERGE, LXsTYPE_BOOLEAN);
    dyna_Add(ATTRs_MERGEDIST, LXsTYPE_DISTANCE);
    dyna_Add(ATTRs_FLAT, LXsTYPE_BOOLEAN);
    dyna_Add(ATTRs_ALIGN, LXsTYPE_ANGLE);
    dyna_Add(ATTRs_CONNECT, LXsTYPE_BOOLEAN);

    tool_Reset();

    CLxUser_PacketService packet_service;
    if (!packet_service.NewVectorType(LXsCATEGORY_TOOL, vector_type))
        throw LXe_FAILED;
    check(packet_service.AddPacket(vector_type, LXsP_TOOL_SUBJECT2, LXfVT_GET));

    offset_subject = packet_service.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SUBJECT2);
    offset_screen = packet_service.GetOffset(LXsCATEGORY_TOOL, LXsP_TOOL_SCREEN_EVENT);
    select_mode = mesh_service.SetMode(LXsMARK_SELECT);

    CLxUser_SceneService scene_service;
    item_type = scene_service.ItemType(SRVNAME_TOOL ".item");
}

void CInsetTool::tool_Reset()
{
    dyna_Value(ATTRi_MODE).SetInt(MODE_SAFE);
    dyna_Value(ATTRi_SIZE).SetFlt(0.15);
    dyna_Value(ATTRi_THRESHOLD).SetFlt(0.01);
    dyna_Value(ATTRi_MERGE).SetInt(0);
    dyna_Value(ATTRi_MERGEDIST).SetFlt(0.04);
    dyna_Value(ATTRi_FLAT).SetInt(0);
    dyna_Value(ATTRi_ALIGN).SetFlt(0.0);
    dyna_Value(ATTRi_CONNECT).SetInt(1);
}

LXtObjectID CInsetTool::tool_VectorType()
{
    return vector_type.m_loc;
}

const char* CInsetTool::tool_Order()
{
    return LXs_ORD_ACTR;
}

LXtID4 CInsetTool::tool_Task()
{
    return LXi_TASK_ACTR;
}

LxResult CInsetTool::tool_GetOp(void** ppvObj, unsigned /*flags*/)
{
    CLxSpawner<CInsetToolOp> spawner(SRVNAME_TOOLOP);
    CInsetToolOp*            toolop = spawner.Alloc(ppvObj);
    if (!toolop)
        return LXe_FAILED;

    dyna_Value(ATTRi_MODE).GetInt(&toolop->mode);
    dyna_Value(ATTRi_SIZE).GetFlt(&toolop->size);
    dyna_Value(ATTRi_THRESHOLD).GetFlt(&toolop->threshold);

    int value = 0;
    dyna_Value(ATTRi_MERGE).GetInt(&value);
    toolop->merge = value != 0;
    dyna_Value(ATTRi_MERGEDIST).GetFlt(&toolop->merge_dist);

    dyna_Value(ATTRi_FLAT).GetInt(&value);
    toolop->flat = value != 0;
    dyna_Value(ATTRi_ALIGN).GetFlt(&toolop->align);
    dyna_Value(ATTRi_CONNECT).GetInt(&value);
    toolop->connect = value != 0;

    toolop->offset_subject = offset_subject;
    toolop->select_mode = select_mode;
    return LXe_OK;
}

LXtTagInfoDesc CInsetTool::descInfo[] = {
    { LXsTOOL_PMODEL, "." },
    { LXsTOOL_USETOOLOP, "." },
    { LXsPMODEL_SELECTIONTYPES, LXsSELOP_TYPE_POLYGON },
    { LXsPMODEL_NOTRANSFORM, "." },
    { 0 }
};

unsigned CInsetTool::tmod_Flags()
{
    return LXfTMOD_I0_INPUT;
}

LxResult CInsetTool::tmod_Enable(ILxUnknownID obj)
{
    CLxUser_Message msg(obj);
    if (!TestPolygon())
    {
        msg.SetCode(LXe_CMD_DISABLED);
        msg.SetMessage(SRVNAME_TOOL, "NoPolygon", 0);
        return LXe_DISABLED;
    }
    return LXe_OK;
}

LxResult CInsetTool::tmod_Down(ILxUnknownID vts, ILxUnknownID adjust)
{
    dyna_Value(ATTRi_SIZE).GetFlt(&m_size0);
    int x, y;
    auto viewId = s_v3d.Mouse (&x, &y);
    s_v3d.View (viewId, m_view3D);
    return LXe_TRUE;
}

void CInsetTool::tmod_Move(ILxUnknownID vts, ILxUnknownID adjust)
{
    CLxUser_AdjustTool at(adjust);
    CLxUser_VectorStack vec(vts);
    LXpToolScreenEvent* spak = static_cast<LXpToolScreenEvent*>(vec.Read(offset_screen));

    double size = m_size0 + (spak->cx - spak->px) * m_view3D.PixelSize();
    if (size < 0.0)
        size = 0.0;
    at.SetFlt(ATTRi_SIZE, size);
}

void CInsetTool::atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints)
{
    switch (index)
    {
        case ATTRi_SIZE:
            hints.MinFloat(0.0001);
            break;
        case ATTRi_THRESHOLD:
            hints.MinFloat(0.0);
            hints.MaxFloat(1.0);
            break;
        case ATTRi_MERGEDIST:
            hints.MinFloat(0.0);
            break;
        default:
            break;
    }
}

LxResult CInsetTool::atrui_DisableMsg (unsigned int index, ILxUnknownID msg)
{
    CLxUser_Message		 message (msg);

    int mode;
    dyna_Value(ATTRi_MODE).GetInt(&mode);

    switch (index) {
        case ATTRi_FLAT:
        case ATTRi_ALIGN:
        case ATTRi_CONNECT:
            if (mode != MODE_PADDING) {
                message.SetCode (LXe_DISABLED);
                message.SetMessage ("tool.smartInset", "OnlyPaddingMode", 0);
                return LXe_DISABLED;
            }
            break;
        case ATTRi_THRESHOLD:
        case ATTRi_MERGE:
        case ATTRi_MERGEDIST:
            if (mode != MODE_SAFE) {
                message.SetCode (LXe_DISABLED);
                message.SetMessage ("tool.smartInset", "OnlySafeMode", 0);
                return LXe_DISABLED;
            }
            break;
    }
    return LXe_OK;
}

LxResult CInsetTool::cui_Enabled(const char* channelName, ILxUnknownID /*msg*/, ILxUnknownID item, ILxUnknownID read)
{
    CLxUser_Item       user_item(item);
    CLxUser_ChannelRead channel_read(read);
    const int mode = channel_read.IValue(user_item, ATTRs_MODE);
    const std::string name(channelName);

    if (mode == MODE_SAFE &&
        (name == ATTRs_FLAT || name == ATTRs_ALIGN || name == ATTRs_CONNECT))
        return LXe_CMD_DISABLED;
    if (mode == MODE_PADDING &&
        (name == ATTRs_THRESHOLD || name == ATTRs_MERGE || name == ATTRs_MERGEDIST))
        return LXe_CMD_DISABLED;

    return LXe_OK;
}

LxResult CInsetTool::cui_DependencyCount(const char* channelName, unsigned* count)
{
    const std::string name(channelName);
    if (name == ATTRs_THRESHOLD || name == ATTRs_MERGE || name == ATTRs_MERGEDIST ||
        name == ATTRs_FLAT || name == ATTRs_ALIGN || name == ATTRs_CONNECT)
        count[0] = 1;
    else
        count[0] = 0;

    return LXe_OK;
}

LxResult CInsetTool::cui_DependencyByIndex(const char* channelName, unsigned index, LXtItemType* depItemType, const char** depChannel)
{
    if (index != 0)
        return LXe_OUTOFBOUNDS;

    const std::string name(channelName);
    if (name != ATTRs_THRESHOLD && name != ATTRs_MERGE && name != ATTRs_MERGEDIST &&
        name != ATTRs_FLAT && name != ATTRs_ALIGN && name != ATTRs_CONNECT)
        return LXe_OUTOFBOUNDS;

    depItemType[0] = item_type;
    depChannel[0] = ATTRs_MODE;
    return LXe_OK;
}

bool CInsetTool::TestPolygon()
{
    CLxUser_LayerScan scan;
    if (!layer_service.BeginScan(LXf_LAYERSCAN_ACTIVE | LXf_LAYERSCAN_MARKPOLYS, scan))
        return false;

    unsigned layer_count = 0;
    if (LXx_FAIL(scan.Count(&layer_count)))
        return false;

    CLxUser_Polygon polygon;
    for (unsigned i = 0; i < layer_count; ++i)
    {
        CLxUser_Mesh mesh;
        if (LXx_FAIL(scan.BaseMeshByIndex(i, mesh)))
            continue;

        polygon.fromMesh(mesh);
        unsigned polygon_count = 0;
        if (LXx_FAIL(mesh.PolygonCount(&polygon_count)))
            continue;

        for (unsigned p = 0; p < polygon_count; ++p)
        {
            if (LXx_OK(polygon.SelectByIndex(p)) && polygon.TestMarks(select_mode) == LXe_TRUE)
            {
                scan.Apply();
                return true;
            }
        }
    }

    scan.Apply();
    return false;
}

void CInsetTool::initialize()
{
    CLxGenericPolymorph* server = new CLxPolymorph<CInsetTool>;
    server->AddInterface(new CLxIfc_Tool<CInsetTool>);
    server->AddInterface(new CLxIfc_ToolModel<CInsetTool>);
    server->AddInterface(new CLxIfc_Attributes<CInsetTool>);
    server->AddInterface(new CLxIfc_AttributesUI<CInsetTool>);
    server->AddInterface(new CLxIfc_ChannelUI<CInsetTool>);
    server->AddInterface(new CLxIfc_StaticDesc<CInsetTool>);
    thisModule.AddServer(SRVNAME_TOOL, server);

    server = new CLxPolymorph<CInsetToolOp>;
    server->AddInterface(new CLxIfc_ToolOperation<CInsetToolOp>);
    lx::AddSpawner(SRVNAME_TOOLOP, server);
}

LxResult CInsetToolOp::top_Evaluate(ILxUnknownID vts)
{
    CLxUser_VectorStack vector_stack(vts);
    if (!vector_stack.ReadObject(offset_subject, subject))
        return LXe_FAILED;

    CLxUser_LayerScan scan;
    if (!subject.BeginScan(LXf_LAYERSCAN_EDIT_POLYS, scan))
        return LXe_FAILED;

    unsigned layer_count = 0;
    check(scan.Count(&layer_count));

    SafeInset::SafeInsetParams safe_params;
    safe_params.inset_size = size;
    safe_params.threshold = threshold;
    safe_params.merge = merge;
    safe_params.merge_dist = merge_dist;

    PaddingInset::PaddingInsetParams padding_params;
    padding_params.inset_size = size;
    padding_params.flat = flat;
    padding_params.align = align;
    padding_params.connect = connect;

    for (unsigned i = 0; i < layer_count; ++i)
    {
        CLxUser_Mesh base_mesh, edit_mesh;
        check(scan.BaseMeshByIndex(i, base_mesh));
        check(scan.EditMeshByIndex(i, edit_mesh));

        CLxUser_Point point(edit_mesh);
        CLxUser_Polygon polygon(edit_mesh);
        //check(point.fromMesh(edit_mesh));
        //check(polygon.fromMesh(edit_mesh));

        std::vector<LXtPolygonID> selected_polys;
        unsigned polygon_count = 0;
        check(base_mesh.PolygonCount(&polygon_count));
        for (unsigned p = 0; p < polygon_count; ++p)
        {
            if (LXx_OK(polygon.SelectByIndex(p)) && polygon.TestMarks(select_mode) == LXe_TRUE)
                selected_polys.push_back(polygon.ID());
        }

        if (selected_polys.empty())
            continue;

        std::vector<LXtPolygonID> top_polys;
        for (LXtPolygonID polygon_id : selected_polys)
        {
            if (LXx_FAIL(polygon.Select(polygon_id)))
                continue;

            std::vector<LXtPolygonID> created_polys;
            if (mode == MODE_SAFE)
                SafeInset::ProcessPolygon(edit_mesh, polygon, point, safe_params, created_polys, top_polys);
            else
                PaddingInset::ProcessPolygon(edit_mesh, polygon, point, padding_params, created_polys, top_polys);
        }

        for (LXtPolygonID top_id : top_polys)
        {
            if (LXx_OK(polygon.Select(top_id)))
                polygon.SetMarks(select_mode);
        }

        check(scan.SetMeshChange(i, LXf_MESHEDIT_GEOMETRY));
    }

    check(scan.Apply());
    return LXe_OK;
}
