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

#include "inset_command.hpp"
#include "padding_inset.hpp"
#include "safe_inset.hpp"

using namespace lx_err;

CInsetCommand::CInsetCommand()
{
    static const LXtTextValueHint inset_modes[] = { { MODE_SAFE, "safe" }, { MODE_PADDING, "padding" }, { 0, "=inset_mode" }, 0 };

    dyna_Add(ATTRs_MODE, LXsTYPE_INTEGER);
    dyna_SetHint(ATTRi_MODE, inset_modes);

    dyna_Add(ATTRs_SIZE, LXsTYPE_DISTANCE);

    dyna_Add(ATTRs_THRESHOLD, LXsTYPE_FLOAT);

    dyna_Add(ATTRs_MERGE, LXsTYPE_BOOLEAN);

    dyna_Add(ATTRs_MERGEDIST, LXsTYPE_DISTANCE);

    dyna_Add(ATTRs_FLAT, LXsTYPE_BOOLEAN);

    dyna_Add(ATTRs_ALIGN, LXsTYPE_ANGLE);

    dyna_Add(ATTRs_CONNECT, LXsTYPE_BOOLEAN);

    check(msh_S.ModeCompose("select", NULL, &select_mode));
}

void CInsetCommand::initialize()
{
    printf("** initialize **\n");
    CLxGenericPolymorph* srv = new CLxPolymorph<CInsetCommand>;
    srv->AddInterface(new CLxIfc_Command<CInsetCommand>);
    srv->AddInterface(new CLxIfc_Attributes<CInsetCommand>);
    srv->AddInterface(new CLxIfc_AttributesUI<CInsetCommand>);
    lx::AddServer(SRVNAME_COMMAND, srv);
}

int CInsetCommand::basic_CmdFlags()
{
    return LXfCMD_MODEL | LXfCMD_UNDO;
}

bool CInsetCommand::basic_Enable(CLxUser_Message& /*msg*/)
{
    int      flags = 0;
    unsigned count = 0;

    check(lyr_S.SetScene(0));
    check(lyr_S.Count(&count));

    for (unsigned i = 0; i < count; ++i)
    {
        check(lyr_S.Flags(i, &flags));
        if (flags & LXf_LAYERSCAN_ACTIVE)
            return true;
    }

    return false;
}

LxResult CInsetCommand::cmd_DialogInit(void)
{
    printf("** cmd_DialogInit **\n");
    if (!LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_MODE)))
        attr_SetInt(ATTRi_MODE, MODE_SAFE);

    if (!LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_SIZE)))
        attr_SetFlt(ATTRi_SIZE, 0.15);

    if (!LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_THRESHOLD)))
        attr_SetFlt(ATTRi_THRESHOLD, 0.01);

    if (!LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_MERGE)))
        attr_SetInt(ATTRi_MERGE, 0);

    if (!LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_MERGEDIST)))
        attr_SetFlt(ATTRi_MERGEDIST, 0.04);

    if (!LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_FLAT)))
        attr_SetInt(ATTRi_FLAT, 0);

    if (!LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_ALIGN)))
        attr_SetFlt(ATTRi_ALIGN, 0.0);

    if (!LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_CONNECT)))
        attr_SetInt(ATTRi_CONNECT, 1);

    return LXe_OK;
}

void CInsetCommand::atrui_UIHints2(unsigned int index, CLxUser_UIHints& hints)
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

LxResult CInsetCommand::cmd_ArgEnable(unsigned int arg)
{
    int mode = MODE_SAFE;
    if (LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_MODE)))
        attr_GetInt(ATTRi_MODE, &mode);

    if (mode == MODE_SAFE)
    {
        if (arg == ATTRi_FLAT || arg == ATTRi_ALIGN || arg == ATTRi_CONNECT)
            return LXe_CMD_DISABLED;
    }
    else if (mode == MODE_PADDING)
    {
        if (arg == ATTRi_THRESHOLD || arg == ATTRi_MERGE || arg == ATTRi_MERGEDIST)
            return LXe_CMD_DISABLED;
    }

    return LXe_OK;
}

void CInsetCommand::basic_Execute(unsigned int /*flags*/)
{
    printf("** Inset: start **\n");
    CLxUser_LayerScan scan;
    unsigned          nLayers = 0;

    int mode = MODE_SAFE;
    if (LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_MODE)))
        attr_GetInt(ATTRi_MODE, &mode);

    SafeInset::SafeInsetParams safe_params;
    if (LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_SIZE)))
        attr_GetFlt(ATTRi_SIZE, &safe_params.inset_size);
    if (LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_THRESHOLD)))
        attr_GetFlt(ATTRi_THRESHOLD, &safe_params.threshold);
    if (LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_MERGE)))
    {
        int merge_val = 0;
        attr_GetInt(ATTRi_MERGE, &merge_val);
        safe_params.merge = (merge_val != 0);
    }
    if (LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_MERGEDIST)))
        attr_GetFlt(ATTRi_MERGEDIST, &safe_params.merge_dist);

    PaddingInset::PaddingInsetParams padding_params;
    if (LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_SIZE)))
        padding_params.inset_size = safe_params.inset_size;
    if (LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_FLAT)))
    {
        int flat_val = 0;
        attr_GetInt(ATTRi_FLAT, &flat_val);
        padding_params.flat = (flat_val != 0);
    }
    if (LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_ALIGN)))
        attr_GetFlt(ATTRi_ALIGN, &padding_params.align);
    if (LXxCMDARG_ISSET(dyna_GetFlags(ATTRi_CONNECT)))
    {
        int connect_val = 1;
        attr_GetInt(ATTRi_CONNECT, &connect_val);
        padding_params.connect = (connect_val != 0);
    }

    check(lyr_S.BeginScan(LXf_LAYERSCAN_EDIT_POLYS, scan));
    check(scan.Count(&nLayers));

    for (unsigned i = 0; i < nLayers; ++i)
    {
        CLxUser_Mesh base_mesh, edit_mesh;
        check(scan.BaseMeshByIndex(i, base_mesh));
        check(scan.EditMeshByIndex(i, edit_mesh));

        CLxUser_Point   point;
        CLxUser_Polygon polygon;
        point.fromMesh(edit_mesh);
        polygon.fromMesh(edit_mesh);

        // Collect all selected polygons in this layer
        std::vector<LXtPolygonID> selected_polys;
        unsigned                  nPolys = 0;
        base_mesh.PolygonCount(&nPolys);
        for (unsigned p = 0; p < nPolys; ++p)
        {
            if (LXx_OK(polygon.SelectByIndex(p)) && (polygon.TestMarks(select_mode) == LXe_TRUE))
            {
                polygon.SelectByIndex(p);
                selected_polys.push_back(polygon.ID());
            }
        }
        printf("** Inset: selected_polys: %zu mode %d **\n", selected_polys.size(), mode);

        if (selected_polys.empty())
            continue;

        std::vector<LXtPolygonID> created_polys;
        std::vector<LXtPolygonID> top_polys;

        for (LXtPolygonID polyID : selected_polys)
        {
            LxResult rc = polygon.Select(polyID);
            printf("** Inset: rc (%d) **\n", LXx_FAIL(rc));

            if (mode == MODE_SAFE)
            {
                SafeInset::ProcessPolygon(edit_mesh, polygon, point, safe_params, created_polys, top_polys);
            }
            else
            {
                PaddingInset::ProcessPolygon(edit_mesh, polygon, point, padding_params, created_polys, top_polys);
            }
        }

        // Set selection to newly created top faces
        for (LXtPolygonID topID : top_polys)
        {
            if (LXx_OK(polygon.Select(topID)))
            {
                polygon.SetMarks(select_mode);
            }
        }

        scan.SetMeshChange(i, LXf_MESHEDIT_GEOMETRY);
    }

    scan.Apply();
}
