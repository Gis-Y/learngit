#ifndef _Apex_DxfRead_hxx
#define _Apex_DxfRead_hxx
#pragma warning(disable : 4996)
#pragma warning(disable : 4819)


#include <vector>
#include <string>
#include <map>
#include <set>
#include <math.h>
#include <cstring>
#include <iostream>
#include <fstream>
#include <string>
#include <locale>
#include <codecvt>
#include <list>
#include <tuple>
#include <algorithm>
#include <limits>
#include <cmath>
#include <fstream>
#include <sstream>

using namespace std;

#include "../DxfCore/dl_dxf.h"
#include "DxfCoreData.hxx"
#include "DxfReaderEx.hxx"

dxf::core::TEntityContainer g_objDxfEntityContainer;

using namespace dxf::core;

class Apex_DxfRead
{

public:
    //// 当前工况得图纸信息
    // StartPoint, EndPoint, m_strOwnerID, m_strLayerName, m_iColorIndex, mid, sid
    std::vector<std::tuple<point_t, point_t, std::string, std::string, int, int, int>>  vec_LatestLine;
    // Point, m_dRadius, m_strOwnerID, m_strLayerName, m_iColorIndex, mid, sid
    std::vector<std::tuple<point_t, double, std::string, std::string, int, int, int>>  vec_LatestCircle;
    // <StartPoint, EndPoint>, m_strOwnerID, m_strLayerName, m_iColorIndex, mid, sid
    std::vector<std::tuple<std::vector<std::pair<point_t, point_t>>, std::string, std::string, int, int, int>>  vec_LatestPolyline;
    // InsertPoint, m_dHeight, m_dXScaleFactor, m_hJustification, m_vJustification, m_strText.c_str(), m_dRotateAngle, m_ptInsert.angle, m_strOwnerID; m_strLayerName; m_iColorIndex, mid, sid
    std::vector<std::tuple<point_t, double, double, int, int, std::string, double, double, std::string, std::string, int, int, int>> vec_LatestText;
    // InsertPoint, m_dHeight; m_dWidth; m_iAttachmentPoint; m_dDrawingDirection; m_dLineSpacingStyle; m_dLineSpacingFactor; m_strText; m_strStyle; m_dAngle; m_strOwnerID; m_strLayerName; m_iColorIndex, mid, sid
    std::vector<std::tuple<point_t, double, double, int, int, int, double, std::string, std::string, double, std::string, std::string, int, int, int>> vec_LatestMText;
    // StartPoint, EndPoint, Text
    std::vector<std::tuple<point_t, point_t, std::string>> vec_LatestDimAligned;
    // m_ptCenter, m_dRadius, m_dStartAngle, m_dEndAngle, m_strOwnerID, m_strLayerName, m_iColorIndex, mid, sid
    std::vector< std::tuple< point_t, double, double, double, std::string, std::string, int, int, int > > vec_LatestArc;



public:
    //// 初始渲染
    void Render();

    //// 输入一个DXF图纸得地址，然后对图纸进行解析
    void Import( std::string strDxfFilename );

    //// 清空文件路径内读取得图纸信息
    void Clear();
};
Apex_DxfRead class_Apex_DxfRead;









//// 初始渲染
void Apex_DxfRead::
Render()
{
    uint32_t arr_idx = 0;
//Begin---has be push_back
    int iLineIndex = 0;
    g_objDxfEntityContainer.for_each_line([&](dxf::core::line_t line) 
    {
        double pt1[] = {line.m_ptStart.x, line.m_ptStart.y, line.m_ptStart.z};
        double pt2[] = {line.m_ptEnd.x, line.m_ptEnd.y, line.m_ptEnd.z};

        int mid = static_cast<int>(dxf::core::LINE);
        int sid  = iLineIndex++;

        vec_LatestLine.push_back( { {pt1[0], pt1[1], pt1[2]}, {pt2[0], pt2[1], pt2[2]}, line.m_strOwnerID, line.m_strLayerName, line.m_iColorIndex, mid, sid } );
    });


    int iTextIndex = 0;
    g_objDxfEntityContainer.for_each_text([&](dxf::core::text_t text) 
    {
        double pt[] = {text.m_ptInsert.x, text.m_ptInsert.y, text.m_ptInsert.z};

        std::string codepage = "ANSI-936";
        if(g_objDxfEntityContainer.isVariableExist("$DWGCODEPAGE"))
        {
            codepage = g_objDxfEntityContainer.getVariable<std::string>("$DWGCODEPAGE");
        }

        int mid = static_cast<int>(dxf::core::TEXT);
        int sid  = iTextIndex++;

        vec_LatestText.push_back( 
                                    {
                                        text.m_ptInsert,
                                        text.m_dHeight, text.m_dXScaleFactor, text.m_hJustification,
                                        text.m_vJustification, text.m_strText.c_str(),
                                        text.m_dRotateAngle, text.m_ptInsert.angle,
                                        text.m_strOwnerID, 
                                        text.m_strLayerName, 
                                        text.m_iColorIndex,
                                        mid, sid
                                    }
                                );
    });

    int iPolylineIndex = 0;
    g_objDxfEntityContainer.for_each_polyline([&](dxf::core::polyline_t polyline) 
    {
        std::vector<double> pts;
        std::vector<int> lines;

        int mid = static_cast<int>(dxf::core::POLYLINE);
        int sid = iPolylineIndex++;

        // 包含线条的子容器
        std::vector<std::pair<dxf::core::point_t, dxf::core::point_t>> vec_LatestPolylineTemp;

        for (size_t Order0 = 0; Order0 < polyline.m_vecPoints.size(); ++Order0) 
        {
            
            const dxf::core::point_t& point = polyline.m_vecPoints[Order0];

            if (Order0 < polyline.m_vecPoints.size() - 1) 
            {
                lines.push_back(Order0);
                lines.push_back(Order0 + 1);
            }
            
            if(Order0 < polyline.m_vecPoints.size() - 1)
            {
                const dxf::core::point_t& point_next = polyline.m_vecPoints[Order0 + 1];

                vec_LatestPolylineTemp.push_back( { {point.x, point.y, point.z}, {point_next.x, point_next.y, point_next.z} } );
            }
        }
        if( vec_LatestPolylineTemp.size() != 0 ) vec_LatestPolyline.push_back( { vec_LatestPolylineTemp, polyline.m_strOwnerID, polyline.m_strLayerName, polyline.m_iColorIndex, mid, sid } );
    });

    int iMTextIndex = 0;
    g_objDxfEntityContainer.for_each_mtext([&](dxf::core::mtext_t mtext) 
    {
        double pt[] = {mtext.m_ptInsert.x, mtext.m_ptInsert.y, mtext.m_ptInsert.z};

        //comx::dxf::GetColorByIndex(mtext.m_iColorIndex, r, g, b);
        int mid = static_cast<int>(dxf::core::MTEXT);
        int sid  = iMTextIndex++;

        std::string codepage = "ANSI-936";//ANSI_1252
        if(g_objDxfEntityContainer.isVariableExist("$DWGCODEPAGE"))
        {
                codepage = g_objDxfEntityContainer.getVariable<std::string>("$DWGCODEPAGE");
        }

        vec_LatestMText.push_back(  {   mtext.m_ptInsert, mtext.m_dHeight, mtext.m_dWidth, mtext.m_iAttachmentPoint, mtext.m_dDrawingDirection, 
                                        mtext.m_dLineSpacingStyle, mtext.m_dLineSpacingFactor, mtext.m_strText, mtext.m_strStyle, 
                                        mtext.m_dAngle, mtext.m_strOwnerID, mtext.m_strLayerName, mtext.m_iColorIndex, mid,sid
                                    }
                                );
    });

    int iCircleIndex = 0;
    g_objDxfEntityContainer.for_each_circle([&](dxf::core::circle_t circle) 
    {
        double pt[] = {circle.m_ptCenter.x, circle.m_ptCenter.y, circle.m_ptCenter.z};

        int mid = static_cast<int>(dxf::core::CIRCLE);
        int sid  = iCircleIndex++;

        vec_LatestCircle.push_back( { circle.m_ptCenter, circle.m_dRadius, circle.m_strOwnerID, circle.m_strLayerName, circle.m_iColorIndex, mid, sid } );
    });

    int dimIndex = 0;
    g_objDxfEntityContainer.for_each_dimAligned([&](dxf::core::dimAligned_t  dimAligned) 
    {
        vec_LatestDimAligned.push_back( { {dimAligned.m_pt2.x, dimAligned.m_pt2.y}, {dimAligned.m_pt3.x, dimAligned.m_pt3.y}, dimAligned.m_strText } );
    });

    int iArcIndex = 0;
    g_objDxfEntityContainer.for_each_arc([&](dxf::core::arc_t arc) 
    {
        double pt[] = {arc.m_ptCenter.x, arc.m_ptCenter.y, arc.m_ptCenter.z};

        int mid = static_cast<int>(dxf::core::ARC);
        int sid  = iArcIndex++;

        vec_LatestArc.push_back( { {arc.m_ptCenter.x, arc.m_ptCenter.y, arc.m_ptCenter.z}, arc.m_dRadius, arc.m_dStartAngle, arc.m_dEndAngle, arc.m_strOwnerID, arc.m_strLayerName, arc.m_iColorIndex, mid, sid } );        
    });
    
//End---has be push_back

    int iVertexIndex = 0;
    g_objDxfEntityContainer.for_each_vertex([&](dxf::core::vertex_t vertex) 
    {
        vector<double> pts_vec = {vertex.m_ptVertex.x, vertex.m_ptVertex.y, vertex.m_ptVertex.z};
        vector<int> vertex_vec = {0};

        int mid = static_cast<int>(dxf::core::VERTEX);
        int sid  = iVertexIndex++;
    });


    int iSolidIndex = 0;
    g_objDxfEntityContainer.for_each_solid([&](dxf::core::solid_t solid) 
    {
        double pt1[] = {solid.m_pt1.x, solid.m_pt1.y, solid.m_pt1.z};
        double pt2[] = {solid.m_pt2.x, solid.m_pt2.y, solid.m_pt2.z};
        double pt3[] = {solid.m_pt3.x, solid.m_pt3.y, solid.m_pt3.z};
        double pt4[] = {solid.m_pt4.x, solid.m_pt4.y, solid.m_pt4.z};

        int mid = static_cast<int>(dxf::core::SOLID);
        int sid  = iSolidIndex++;
    });
        
    int iElipseIndex = 0;
    g_objDxfEntityContainer.for_each_elipse([&](dxf::core::elipse_t elipse) 
    {
        double pt[] = {elipse.m_ptCenter.x, elipse.m_ptCenter.y, elipse.m_ptCenter.z};
        double major_axis[] = {elipse.m_dirMajor.x, elipse.m_dirMajor.y, elipse.m_dirMajor.z};
        double minor_axis[] = {elipse.m_dirMinor.x, elipse.m_dirMinor.y, elipse.m_dirMinor.z};

        int mid = static_cast<int>(dxf::core::ELLIPSE);
        int sid  = iElipseIndex++;
    });

    int iFaceIndex = 0;
    g_objDxfEntityContainer.for_each_face([&](dxf::core::face_3d_t face) 
    {
        double pt1[] = {face.m_pt1.x, face.m_pt1.y, face.m_pt1.z};
        double pt2[] = {face.m_pt2.x, face.m_pt2.y, face.m_pt2.z};
        double pt3[] = {face.m_pt3.x, face.m_pt3.y, face.m_pt3.z};
        double pt4[] = {face.m_pt4.x, face.m_pt4.y, face.m_pt4.z};

        int mid = static_cast<int>(dxf::core::FACE);
        int sid  = iFaceIndex++;
    });
}










// 输入一个DXF图纸得地址，然后对图纸进行解析
void Apex_DxfRead::
Import( std::string strDxfFilename )
{
    auto &core = g_objDxfEntityContainer;

    core.clear();
            
    dxf::reader::TDxfFilter filter(strDxfFilename);

    filter.updateLines([&](auto line) {
        core.push_back(line);
    });
    filter.updateVertexs([&](auto vertex) {
        core.push_back(vertex);
    });
    filter.updateArcs([&](auto arc) {
        core.push_back(arc);
    });
    filter.updateCircles([&](auto circle) {
        core.push_back(circle);
    });
    filter.updateFace3ds([&](auto face) {
        core.push_back(face);
    });
    filter.updateElipses([&](auto elipse) {
        core.push_back(elipse);
    });
    filter.updatePolylines([&](auto polyline) {
        core.push_back(polyline);
    });
    filter.updateSolids([&](auto solid) {
        core.push_back(solid);
    });
    filter.updateDimAligneds([&](auto dimAligned) {
        core.push_back(dimAligned);
    });
    filter.updateTexts([&](auto text) {
        core.push_back(text);
    });
    filter.updateMTexts([&](auto mtext) {
        core.push_back(mtext);
    });
    filter.updateBlocks([&](auto block) {
        core.push_back(block);
    });
    filter.updateLayers([&](auto layer) {
        core.push_back(layer);
    });
    filter.updateVariables([&](auto key, auto var) {
        core.insertVariable(key, var);
    });
}










//// 清空文件路径内读取得图纸信息
void Apex_DxfRead::
Clear()
{
    g_objDxfEntityContainer.clear();
}



#endif