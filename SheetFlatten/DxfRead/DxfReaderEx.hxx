#ifndef _c186475efb4a99bed5f71e8487b85c41_DxfReaderEx_hxx
#define _c186475efb4a99bed5f71e8487b85c41_DxfReaderEx_hxx

//////////////////////////////////////////////////////////////////////////////////////
// put your codes here
#include "../DxfCore/dl_dxf.h"
#include "../DxfCore/dl_writer.h"
#include "../DxfCore/dl_creationinterface.h"
#include "../DxfCore/dl_creationadapter.h"

#include "DxfReaderAlgorithm.hxx"
#include "DxfCoreData.hxx"

#include <functional>
#include <assert.h>
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
using namespace std;
using namespace dxf::core;

namespace dxf {
namespace reader {
static const int ID_CODE = 330;
static int s_BlockID = 2023;
static const int BYBLOCK = 0;
static const int BYLAYER  = 256;

class TDxfFilter : public DL_CreationAdapter
{
public:
        TDxfFilter(){};
        ~TDxfFilter() noexcept override {};
        TDxfFilter(const std::string &fileName) : mDxf(new DL_Dxf())
        {
                mDxf->in(fileName, this);
                
                if(m_mapLayerColorIndex.find("0") == m_mapLayerColorIndex.end())
                {
                        m_mapLayerColorIndex["0"] = 7;
                        m_mapLayerVisible["0"] = true;
                }
        }
public:
        template<typename Functor>
        void updateLines(Functor func)
        {
                int cnt = 0;
                for_each(m_vecLines.begin(), m_vecLines.end(), [&](auto line) {
                        dxf::core::point_t spt = line.m_ptStart, ept = line.m_ptEnd;
                        spt.color_index = line.m_iColorIndex;
                        ept.color_index = line.m_iColorIndex;

                        std::vector<dxf::core::point_t> start_pts, end_pts;
                        bool isSptInModelSpace = calcPoint(spt, line.m_strOwnerID, line.m_strLayerName, start_pts);
                        bool isEptInModelSpace = calcPoint(ept, line.m_strOwnerID, line.m_strLayerName, end_pts);

                        assert(isSptInModelSpace == isEptInModelSpace);
                        assert(start_pts.size() == end_pts.size());

                        if ((isSptInModelSpace || line.m_strOwnerID == "") && m_mapLayerVisible[line.m_strLayerName])
                        {
                                for (std::size_t loop = 0; loop < start_pts.size(); ++loop)
                                {
                                        line.m_ptStart = start_pts[loop];
                                        line.m_ptEnd   = end_pts[loop];

                                        line.m_iColorIndex = line.m_ptStart.color_index;

                                        reviseModelSpaceEntityColorIndex(line);
                                        func(line);
                                }
                        }
                });
        }

        template<typename Functor>
        void updateVertexs(Functor func)
        {
                for_each(m_ptVertexs.begin(), m_ptVertexs.end(), [&](auto vertex) {
                        dxf::core::point_t pt = vertex.m_ptVertex;
                        pt.angle = vertex.m_bulge;  //ke neng xu yao shan le 
                        pt.color_index = vertex.m_iColorIndex;

                        std::vector<dxf::core::point_t> pts;
                        bool isInModelSpace = calcPoint(pt, vertex.m_strOwnerID, vertex.m_strLayerName, pts);

                        if ((isInModelSpace || vertex.m_strOwnerID == "") && m_mapLayerVisible[vertex.m_strLayerName])
                        {
                                for (std::size_t loop = 0; loop < pts.size(); ++loop)
                                {
                                        vertex.m_ptVertex = pts[loop];
                                        vertex.m_iColorIndex = vertex.m_ptVertex.color_index;
                                        reviseModelSpaceEntityColorIndex(vertex);
                                        func(vertex);
                                }
                        }
                });
        }

        template<typename Functor>
        void updatePolylines(Functor func)
        {
                int cnt = 0;
                for_each(m_vecPolylines.begin(), m_vecPolylines.end(), [&](auto polyline) {
                        dxf::core::polyline_t new_polyline = polyline;
                        new_polyline.m_vecPoints.clear();
                        for (auto pt : polyline.m_vecPoints) {
                                pt.color_index = polyline.m_iColorIndex;
                                std::vector<dxf::core::point_t> pts_in_model_space;
                                bool isPtInModelSpace = calcPoint(pt, polyline.m_strOwnerID, polyline.m_strLayerName, pts_in_model_space);
                                if ((isPtInModelSpace || polyline.m_strOwnerID == "") && m_mapLayerVisible[polyline.m_strLayerName]) {
                                        for (auto new_pt : pts_in_model_space) {
                                                new_pt.color_index = pt.color_index;
                                                reviseModelSpaceEntityColorIndex(polyline);
                                                new_polyline.m_vecPoints.push_back(new_pt);
                                        }
                                }
                        }
                        func(new_polyline);
                });
        }
        template<typename Functor>
        void updateSolids(Functor func)
        {
                for_each(m_vecSolids.begin(), m_vecSolids.end(), [&](auto solid) {
                        dxf::core::point_t pt1 = solid.m_pt1;
                        pt1.color_index = solid.m_iColorIndex;

                        dxf::core::point_t pt2 = solid.m_pt2;
                        pt2.color_index = solid.m_iColorIndex;

                        dxf::core::point_t pt3 = solid.m_pt3;
                        pt3.color_index = solid.m_iColorIndex;

                        dxf::core::point_t pt4 = solid.m_pt4;
                        pt4.color_index = solid.m_iColorIndex;

                        vector<dxf::core::point_t> pt1s, pt2s, pt3s, pt4s;

                        bool isInModelSpace = calcPoint(pt1, solid.m_strOwnerID, solid.m_strLayerName, pt1s);
                        calcPoint(pt2, solid.m_strOwnerID, solid.m_strLayerName, pt2s);
                        calcPoint(pt3, solid.m_strOwnerID, solid.m_strLayerName, pt3s);
                        calcPoint(pt4, solid.m_strOwnerID, solid.m_strLayerName, pt4s);

                        if ((isInModelSpace || solid.m_strOwnerID == "") && m_mapLayerVisible[solid.m_strLayerName])
                        {
                                for (std::size_t loop = 0; loop < pt1s.size(); ++loop)
                                {
                                        solid.m_pt1 = pt1s[loop];
                                        solid.m_pt2 = pt2s[loop];
                                        solid.m_pt3 = pt3s[loop];
                                        solid.m_pt4 = pt4s[loop];
                                        solid.m_iColorIndex = solid.m_pt1.color_index;

                                        reviseModelSpaceEntityColorIndex(solid);
                                        func(solid);
                                }
                        }
                });
        }

        template<typename Functor>
        void updateDimAligneds(Functor func) {
                for_each(m_vecDimAligneds.begin(), m_vecDimAligneds.end(), [&](auto dimAligned) {
                        // Extract properties from dim_aligned_t
                        dxf::core::point_t pt1 = dimAligned.m_pt1;
                        pt1.color_index = dimAligned.m_iColorIndex;
                        dxf::core::point_t pt2 = dimAligned.m_pt2;
                        pt2.color_index = dimAligned.m_iColorIndex;
                        dxf::core::point_t pt3 = dimAligned.m_pt3;
                        pt3.color_index = dimAligned.m_iColorIndex;
                        dxf::core::point_t ptMText = dimAligned.m_ptMText;
                        ptMText.color_index = dimAligned.m_iColorIndex;

                        std::vector<dxf::core::point_t> pt1s, pt2s, pt3s, ptMTexts;
                        calcPoint(pt1, dimAligned.m_strOwnerID, dimAligned.m_strLayerName, pt1s);
                        calcPoint(pt2, dimAligned.m_strOwnerID, dimAligned.m_strLayerName, pt2s);
                        calcPoint(pt3, dimAligned.m_strOwnerID, dimAligned.m_strLayerName, pt3s);
                        bool isInModelSpace = calcPoint(ptMText, dimAligned.m_strOwnerID, dimAligned.m_strLayerName, ptMTexts);

                        if ((isInModelSpace || dimAligned.m_strOwnerID == "") && m_mapLayerVisible[dimAligned.m_strLayerName]) {
                                for (std::size_t loop = 0; loop < ptMTexts.size(); ++loop) {
                                        // Update dim_aligned_t properties with transformed values
                                        dimAligned.m_pt1 = pt1s[loop];
                                        dimAligned.m_pt2 = pt2s[loop];
                                        dimAligned.m_pt3 = pt3s[loop];
                                        dimAligned.m_ptMText = ptMTexts[loop];
                                        dimAligned.m_iColorIndex = dimAligned.m_ptMText.color_index;

                                        // Apply the operation function to the updated dim_aligned_t
                                        reviseModelSpaceEntityColorIndex(dimAligned);
                                        func(dimAligned);
                                }
                        }
                });
        }


        template<typename Functor>
        void updateArcs(Functor func)
        {
                for_each(m_vecArcs.begin(), m_vecArcs.end(), [&](auto arc) {
                        dxf::core::point_t pt = arc.m_ptCenter;
                        pt.color_index = arc.m_iColorIndex;

                        vector<dxf::core::point_t> pts;
                        bool isInModelSpace = calcPoint(pt, arc.m_strOwnerID, arc.m_strLayerName, pts);

                        if ((isInModelSpace || arc.m_strOwnerID == "") && m_mapLayerVisible[arc.m_strLayerName])
                        {
                                for (std::size_t loop = 0; loop < pts.size(); ++loop)
                                {
                                        arc.m_ptCenter = pts[loop];
                                        arc.m_iColorIndex = arc.m_ptCenter.color_index;

                                        reviseModelSpaceEntityColorIndex(arc);
                                        func(arc);
                                }
                        }
                });
        }

        template<typename Functor>
        void updateCircles(Functor func)
        {
                for_each(m_vecCircles.begin(), m_vecCircles.end(), [&](auto circle) {
                        dxf::core::point_t pt = circle.m_ptCenter;
                        pt.color_index = circle.m_iColorIndex;

                        vector<dxf::core::point_t> pts;
                        bool isInModelSpace = calcPoint(pt, circle.m_strOwnerID, circle.m_strLayerName, pts);

                        if ((isInModelSpace || circle.m_strOwnerID == "") && m_mapLayerVisible[circle.m_strLayerName])
                        {
                                for (std::size_t loop = 0; loop < pts.size(); ++loop)
                                {
                                        circle.m_ptCenter = pts[loop];
                                        circle.m_iColorIndex = circle.m_ptCenter.color_index;

                                        reviseModelSpaceEntityColorIndex(circle);
                                        func(circle);
                                }
                        }
                });
        }

        template<typename Functor>
        void updateFace3ds(Functor func)
        {
                for_each(m_vecFace3ds.begin(), m_vecFace3ds.end(), [&](auto face) {
                        dxf::core::point_t pt1 = face.m_pt1;
                        pt1.color_index = face.m_iColorIndex;

                        dxf::core::point_t pt2 = face.m_pt2;
                        pt2.color_index = face.m_iColorIndex;

                        dxf::core::point_t pt3 = face.m_pt3;
                        pt3.color_index = face.m_iColorIndex;

                        dxf::core::point_t pt4 = face.m_pt4;
                        pt4.color_index = face.m_iColorIndex;

                        vector<dxf::core::point_t> pt1s, pt2s, pt3s, pt4s;

                        bool isInModelSpace = calcPoint(pt1, face.m_strOwnerID, face.m_strLayerName, pt1s);
                        calcPoint(pt2, face.m_strOwnerID, face.m_strLayerName, pt2s);
                        calcPoint(pt3, face.m_strOwnerID, face.m_strLayerName, pt3s);
                        calcPoint(pt4, face.m_strOwnerID, face.m_strLayerName, pt4s);

                        if ((isInModelSpace || face.m_strOwnerID == "") && m_mapLayerVisible[face.m_strLayerName])
                        {
                                for (std::size_t loop = 0; loop < pt1s.size(); ++loop)
                                {
                                        face.m_pt1 = pt1s[loop];
                                        face.m_pt2 = pt2s[loop];
                                        face.m_pt3 = pt3s[loop];
                                        face.m_pt4 = pt4s[loop];
                                        face.m_iColorIndex = face.m_pt1.color_index;

                                        reviseModelSpaceEntityColorIndex(face);
                                        func(face);
                                }
                        }
                });
        }

        template<typename Functor>
        void updateElipses(Functor func)
        {
                for_each(m_vecElipses.begin(), m_vecElipses.end(), [&](auto elipse) {
                        dxf::core::point_t ptCenter = elipse.m_ptCenter;
                        ptCenter.color_index = elipse.m_iColorIndex;

                        vector<dxf::core::point_t> ptCenters;
                        bool isInModelSpace = calcPoint(ptCenter, elipse.m_strOwnerID, elipse.m_strLayerName, ptCenters);

                        if ((isInModelSpace || elipse.m_strOwnerID == "") && m_mapLayerVisible[elipse.m_strLayerName])
                        {
                                for (std::size_t loop = 0; loop < ptCenters.size(); ++loop)
                                {
                                        elipse.m_ptCenter = ptCenters[loop];
                                        elipse.m_iColorIndex = elipse.m_ptCenter.color_index;

                                        reviseModelSpaceEntityColorIndex(elipse);
                                        func(elipse);
                                }
                        }
                });
        }

        template<typename Functor>
        void updateTexts(Functor func)
        {
                for_each(m_vecTexts.begin(), m_vecTexts.end(), [&](auto text) {
                        dxf::core::point_t ptInsert = text.m_ptInsert;
                        ptInsert.color_index = text.m_iColorIndex;

                        vector<dxf::core::point_t> ptInserts;
                        bool isInModelSpace = calcPoint(ptInsert, text.m_strOwnerID, text.m_strLayerName, ptInserts);

                        if ((isInModelSpace || text.m_strOwnerID == "") && m_mapLayerVisible[text.m_strLayerName])
                        {
                                for (std::size_t loop = 0; loop < ptInserts.size(); ++loop)
                                {
                                        text.m_ptInsert = ptInserts[loop];
                                        text.m_iColorIndex = text.m_ptInsert.color_index;

                                        reviseModelSpaceEntityColorIndex(text);
                                        func(text);
                                }
                        }
                });
        }

        template<typename Functor>
        void updateMTexts(Functor func)
        {
                for_each(m_vecMTexts.begin(), m_vecMTexts.end(), [&](auto mtext) {
                        dxf::core::point_t ptInsert = mtext.m_ptInsert;
                        ptInsert.color_index        = mtext.m_iColorIndex;

                        vector<dxf::core::point_t> ptInserts;
                        bool isInModelSpace = calcPoint(ptInsert, mtext.m_strOwnerID, mtext.m_strLayerName, ptInserts);

                        if ((isInModelSpace || mtext.m_strOwnerID == "") && m_mapLayerVisible[mtext.m_strLayerName])
                        {
                                for (std::size_t loop = 0; loop < ptInserts.size(); ++loop)
                                {
                                        mtext.m_ptInsert    = ptInserts[loop];
                                        mtext.m_iColorIndex = mtext.m_ptInsert.color_index;

                                        reviseModelSpaceEntityColorIndex(mtext);
                                        func(mtext);
                                }
                        }
                });
        }


        template<typename Functor>
        void updateBlocks(Functor func)
        {
                for(auto iter = m_mapID2Block.begin(); iter != m_mapID2Block.end(); ++iter)
                {
                        func(dxf::core::block_t{iter->second.m_strBlockName});
                }
        }

        template<typename Functor>
        void updateLayers(Functor func)
        {
                for(auto iter = m_mapLayerColorIndex.begin(); iter != m_mapLayerColorIndex.end(); ++iter)
                {
                        func(dxf::core::layer_t{iter->first, iter->second});
                }
        }

        template<typename Functor>
        void updateVariables(Functor func)
        {
                for (auto iter = m_mapVariable.begin(); iter != m_mapVariable.end(); ++iter)
                {
                        func(iter->first, iter->second);
                }
        }
        
        int index = 0;
        virtual void addLine(const DL_LineData & line)
        {
                if (line.x1 == line.x2 && line.y1 == line.y2 && line.z1 == line.z2) return;

                string strOwnerID = m_strCurrentBlockID;//mDxf->getStringValue(330, "");
                string strLayerName = mDxf->getStringValue(8, "");

                int iColorIndex = mDxf->getIntValue(62, 256);
                
                if(strLayerName == "0")
                {
                        iColorIndex = 2;
                }
                dxf::core::point_t spt = dxf::core::point_t(line.x1, line.y1, line.z1);
                dxf::core::point_t ept = dxf::core::point_t(line.x2, line.y2, line.z2);
                m_vecLines.push_back(dxf::core::line_t{ spt, ept, strOwnerID, strLayerName, iColorIndex });
        }

        virtual void addArc(const DL_ArcData &arc)
        {
                string strLayerName = mDxf->getStringValue(8, "");
                int iColorIndex = mDxf->getIntValue(62, 256);
                m_vecArcs.push_back({ 
                        dxf::core::point_t(arc.cx, arc.cy, arc.cz), 
                        arc.radius, 
                        arc.angle1,
                        arc.angle2, 
                        m_strCurrentBlockID, 
                        strLayerName, 
                        iColorIndex });
        }

        virtual void addArcAlignedText(const DL_ArcAlignedTextData &)
        {
                //nothing.
        }

        virtual void addCircle(const DL_CircleData& circle)
        {
                string strLayerName = mDxf->getStringValue(8, "");
                int iColorIndex = mDxf->getIntValue(62, 256);
                m_vecCircles.push_back({ 
                        dxf::core::point_t(circle.cx, circle.cy, circle.cz), 
                        circle.radius, 
                        m_strCurrentBlockID, 
                        strLayerName, 
                        iColorIndex });
        }

        virtual void addText(const DL_TextData &text)
        {
                string strLayerName = mDxf->getStringValue(8, "");
                int iColorIndex = mDxf->getIntValue(62, 256);

                if(text.hJustification == 0 && text.vJustification == 0)
                {
                        m_vecTexts.push_back({dxf::core::point_t(text.ipx, text.ipy, text.ipz),
                                dxf::core::point_t(text.apx, text.apy, text.apz),
                                text.height,
                                text.xScaleFactor,
                                text.textGenerationFlags,
                                text.hJustification,
                                text.vJustification,
                                text.text,
                                text.style,
                                text.angle,
                                m_strCurrentBlockID,
                                strLayerName,
                                iColorIndex});
                }
                else
                {
                        m_vecTexts.push_back({dxf::core::point_t(text.apx, text.apy, 0.0/*text.apz*/),
                                dxf::core::point_t(text.apx, text.apy, text.apz),
                                text.height,
                                text.xScaleFactor,
                                text.textGenerationFlags,
                                text.hJustification,
                                text.vJustification,
                                text.text,
                                text.style,
                                text.angle,
                                m_strCurrentBlockID,
                                strLayerName,
                                iColorIndex});
                }
        }
        virtual void processCodeValuePair(unsigned int code, const std::string&value) {}
        virtual void endSection() {}
        virtual void addLayer(const DL_LayerData& layer)
        {
                int flags = layer.flags;
                int color_index = mDxf->getIntValue(62, 0);

                m_mapLayerVisible[layer.name] = !(flags == 1 && color_index < 0);
                m_mapLayerColorIndex[layer.name] = color_index;
        }
        virtual void addLinetype(const DL_LinetypeData&) {}
        virtual void addLinetypeDash(double) {}
        virtual void addBlock(const DL_BlockData& block)
        {
                std::string strBlockID = mDxf->getStringValue(330, "undefined");

                if (strBlockID == "undefined")
                {
                        strBlockID = generateBlockID();
                }

                std::string block_name = "";
                std::transform(block.name.begin(), block.name.end(), std::back_inserter(block_name), ::tolower);

                if (block_name.find("model_space") != std::string::npos)
                {
                        m_strCurrentBlockID = m_strModelBlockID = strBlockID;
                }
                else
                {
                        m_strCurrentBlockID = strBlockID;
                }

                int iColorIndex = mDxf->getIntValue(62, 256);

                dxf::core::TBlockNode nodeBlk;
                nodeBlk.m_strBlockName = block.name;
                nodeBlk.m_strBlockID = strBlockID;
                nodeBlk.m_ptBlockBase = dxf::core::point_t(block.bpx, block.bpy, block.bpz);
                nodeBlk.m_iColorIndex = iColorIndex;
                nodeBlk.m_strLayerName = mDxf->getStringValue(8, "");

                m_mapID2Block[strBlockID] = nodeBlk;
        }

        virtual void endBlock() {
                m_strCurrentBlockID = m_strModelBlockID;
        }
        virtual void addTextStyle(const DL_StyleData&) {}

        virtual void addPoint(const DL_PointData &pt)
        {
                dxf::core::point_t point = dxf::core::point_t(pt.x, pt.y, pt.z);
                string strOwnerID = m_strCurrentBlockID/*mDxf->getStringValue(ID_CODE, "")*/;

                string strLayerName = mDxf->getStringValue(8, "");
                int iColorIndex = mDxf->getIntValue(62, 256);

                m_ptVertexs.push_back(dxf::core::vertex_t{point, point.angle,strOwnerID, strLayerName, iColorIndex });
        }

        virtual void add3dFace(const DL_3dFaceData &face)
        {
                string strLayerName = mDxf->getStringValue(8, "");
                int iColorIndex = mDxf->getIntValue(62, 256);
                m_vecFace3ds.push_back({
                        dxf::core::point_t(face.x[0], face.y[0], face.z[0]),
                        dxf::core::point_t(face.x[1], face.y[1], face.z[1]),
                        dxf::core::point_t(face.x[3], face.y[3], face.z[3]),
                        dxf::core::point_t(face.x[2], face.y[2], face.z[2]),
                        face.thickness,
                        m_strCurrentBlockID,
                        strLayerName,
                        iColorIndex });
        }
        virtual void addSolid(const DL_SolidData& solid)
        {
                string strLayerName = mDxf->getStringValue(8, "");
                int iColorIndex = mDxf->getIntValue(62, 256);
                m_vecSolids.push_back({
                        dxf::core::point_t(solid.x[0], solid.y[0], solid.z[0]),
                        dxf::core::point_t(solid.x[1], solid.y[1], solid.z[1]),
                        dxf::core::point_t(solid.x[3], solid.y[3], solid.z[3]),
                        dxf::core::point_t(solid.x[2], solid.y[2], solid.z[2]),
                        solid.thickness,
                        m_strCurrentBlockID,
                        strLayerName,
                        iColorIndex });
        }

        virtual void addXLine(const DL_XLineData&) {}
        virtual void addRay(const DL_RayData&) {}

        virtual void addEllipse(const DL_EllipseData &elipse)
        {
                dxf::core::vector_t major_axis = { elipse.mx, elipse.my, elipse.mz };
                double major_radius = dxf::algorithm::get_model(major_axis);
                double minor_radius = major_radius * elipse.ratio;

                major_axis = dxf::algorithm::normalize_vec(major_axis);

                dxf::core::vector_t stretch_dir = {
                        extrusion->getDirection()[0],
                        extrusion->getDirection()[1],
                        extrusion->getDirection()[2]
                };
                stretch_dir = dxf::algorithm::normalize_vec(stretch_dir);

                dxf::core::vector_t minor_axis = dxf::algorithm::normalize_vec(dxf::algorithm::fork_product(stretch_dir, major_axis));

                string strLayerName = mDxf->getStringValue(8, "");
                int iColorIndex = mDxf->getIntValue(62, 256);

                m_vecElipses.push_back({ dxf::core::point_t(elipse.cx, elipse.cy, elipse.cz), major_radius, minor_radius,  major_axis, minor_axis,  elipse.angle1, elipse.angle2, m_strCurrentBlockID, strLayerName, iColorIndex });
        }

        virtual void addPolyline(const DL_PolylineData &polyline)
        {
                dxf::core::polyline_t new_polyline;
                string strOwnerID = m_strCurrentBlockID;
                string strLayerName = mDxf->getStringValue(8, "");
                int iColorIndex = mDxf->getIntValue(62, 256);
                new_polyline.m_strOwnerID = strOwnerID;
                new_polyline.m_strLayerName = strLayerName;
                new_polyline.m_iColorIndex = iColorIndex;

                m_vecPolylines.push_back(new_polyline);
        }

        virtual void addVertex(const DL_VertexData& vertex) 
        {
                if (!m_vecPolylines.empty()) {
                        dxf::core::point_t pt(vertex.x, vertex.y, vertex.z);
                        double bulge = 0;
                        string strOwnerID = m_strCurrentBlockID;//mDxf->getStringValue(330, "");
                        string strLayerName = mDxf->getStringValue(8, "");
                        int iColorIndex = mDxf->getIntValue(62, 256);
                        pt.color_index = iColorIndex;

                        m_vecPolylines.back().m_vecPoints.push_back(pt);
                        
                        m_ptVertexs.push_back({
                        dxf::core::point_t(vertex.x, vertex.y, vertex.z),
                        vertex.bulge,
                        m_strCurrentBlockID,
                        strLayerName,
                        iColorIndex
                        });
                }
        }

        virtual void addSpline(const DL_SplineData&) {}
        virtual void addControlPoint(const DL_ControlPointData&) {}
        virtual void addFitPoint(const DL_FitPointData&) {}
        virtual void addKnot(const DL_KnotData&) {}

        virtual void addInsert(const DL_InsertData& insert)
        {
                dxf::core::TInsertNode nodeInsert;
                nodeInsert.m_strBlockName = insert.name;
                nodeInsert.m_ptInsertBase = dxf::core::point_t(insert.ipx, insert.ipy, insert.ipz);
                nodeInsert.m_vecInsertScale = dxf::core::point_t(insert.sx, insert.sy, insert.sz);
                nodeInsert.m_vecInsertStretchDirection = dxf::core::point_t(extrusion->getDirection()[0],
                                extrusion->getDirection()[1],
                                extrusion->getDirection()[2]);

                std::string strParentBlockID = mDxf->getStringValue(ID_CODE, "undefined");
                if (strParentBlockID == "undefined")
                {
                        strParentBlockID = m_strCurrentBlockID;
                }

                nodeInsert.m_dRotateAngle = insert.angle;
                nodeInsert.m_strParentBlockID = strParentBlockID;
                nodeInsert.m_strLayerName = mDxf->getStringValue(8, "");
                nodeInsert.m_iColorIndex = mDxf->getIntValue(62, 256);
                m_mapName2Insert[insert.name].push_back(nodeInsert);
        }

        virtual void addMText(const DL_MTextData &mtext)
        {
                string strLayerName = mDxf->getStringValue(8, "");
                int iColorIndex = mDxf->getIntValue(62, 256);

                m_vecMTexts.push_back({dxf::core::point_t(mtext.ipx, mtext.ipy, mtext.ipz),
                        mtext.height,
                        mtext.width,
                        mtext.attachmentPoint,
                        mtext.drawingDirection,
                        mtext.lineSpacingStyle,
                        mtext.lineSpacingFactor,
                        mtext.text,
                        mtext.style,
                        mtext.angle,
                        m_strCurrentBlockID,
                        strLayerName,
                        iColorIndex});
        }
//New Write Begin   
        virtual void addDimAlign(const DL_DimensionData& dimData, const DL_DimAlignedData& alignedData) 
        {
                dxf::core::dimAligned_t dimAlign;

                dimAlign.m_pt1 = point_t(dimData.dpx, dimData.dpy, dimData.dpz);
                dimAlign.m_pt2 = point_t(alignedData.epx1, alignedData.epy1, alignedData.epz1);
                dimAlign.m_pt3 = point_t(alignedData.epx2, alignedData.epy2, alignedData.epz2);
                dimAlign.m_ptMText = point_t(dimData.mpx, dimData.mpy, dimData.mpz); // You need to calculate the text position based on dimData
                dimAlign.m_dAngle = dimData.angle; // Angle in radians
                dimAlign.m_dTextRotation;  // Text rotation angle in radians
                dimAlign.m_AttachmentPoint = dimData.attachmentPoint;
                dimAlign.m_dLineSpacingStyle = dimData.lineSpacingStyle;
                dimAlign.m_dLineSpacingFactor = dimData.lineSpacingFactor;
                dimAlign.m_strText = dimData.text;
                dimAlign.m_strStyle = dimData.style;
                dimAlign.m_strOwnerID = m_strCurrentBlockID; // or obtain the owner ID using appropriate method
                dimAlign.m_strLayerName = mDxf->getStringValue(8, ""); // Obtain layer name from the DXF data
                dimAlign.m_iColorIndex = mDxf->getIntValue(62, 256); // Obtain color index from the DXF data
                // Store the parsed dim_aligned_t data in your data structure or process it as needed
                m_vecDimAligneds.push_back(dimAlign);
        }
        virtual void addDimLinear(const DL_DimensionData&,
                                  const DL_DimLinearData&) {}
        virtual void addDimRadial(const DL_DimensionData&,
                                  const DL_DimRadialData&) {}
        virtual void addDimDiametric(const DL_DimensionData&,
                                     const DL_DimDiametricData&) {}
        virtual void addDimAngular(const DL_DimensionData&,
                                   const DL_DimAngular2LData&) {}
        virtual void addDimAngular3P(const DL_DimensionData&,
                                     const DL_DimAngular3PData&) {}
        virtual void addDimOrdinate(const DL_DimensionData&,
                                    const DL_DimOrdinateData&) {}
//New Write End
        virtual void addAttribute(const DL_AttributeData& attr) {}
        virtual void addLeader(const DL_LeaderData&) {}
        virtual void addLeaderVertex(const DL_LeaderVertexData&) {}
        virtual void addHatch(const DL_HatchData&) {}
        virtual void addTrace(const DL_TraceData&) {}

        virtual void addImage(const DL_ImageData&) {}
        virtual void linkImage(const DL_ImageDefData&) {}
        virtual void addHatchLoop(const DL_HatchLoopData&) {}
        virtual void addHatchEdge(const DL_HatchEdgeData&) {}
        virtual void addXRecord(const std::string&) {}
        virtual void addXRecordString(int, const std::string&) {}
        virtual void addXRecordReal(int, double) {}
        virtual void addXRecordInt(int, int) {}
        virtual void addXRecordBool(int, bool) {}
        virtual void addXDataApp(const std::string&) {}
        virtual void addXDataString(int, const std::string&) {}
        virtual void addXDataReal(int, double) {}
        virtual void addXDataInt(int, int) {}
        virtual void addDictionary(const DL_DictionaryData&) {}
        virtual void addDictionaryEntry(const DL_DictionaryEntryData&) {}
        virtual void addComment(const std::string& comment) {}
        virtual void setVariableVector(const std::string& key, double rx, double ry , double rz, int code) {
                m_mapVariable[key] = { dxf::core::toString(dxf::core::vector_t(rx, ry, rz)), code, dxf::core::variable_t::VECTOR};
        }
        virtual void setVariableString(const std::string& key, const std::string& value, int code) {
                m_mapVariable[key] = { value, code, dxf::core::variable_t::STRING };
	}
        virtual void setVariableInt(const std::string& key, int value, int code) {
                m_mapVariable[key] = { dxf::core::toString(value), code, dxf::core::variable_t::INT};
        }
        virtual void setVariableDouble(const std::string& key, double value, int code) {
                m_mapVariable[key] = { dxf::core::toString(value), code, dxf::core::variable_t::DOUBLE};
        }
#ifdef DL_COMPAT
        virtual void setVariableVector(const char*, double, double, double, int) {}
        virtual void setVariableString(const char*, const char*, int) {}
        virtual void setVariableInt(const char*, int, int) {}
        virtual void setVariableDouble(const char*, double, int) {}
        virtual void processCodeValuePair(unsigned int, char*) {}
        virtual void addComment(const char*) {}
        virtual void addMTextChunk(const char*) {}
#endif
        virtual void endSequence() {}

private:
        void transOCStoWCS(dxf::core::stretch_direction_t stretch_dir, double &x, double &y)
        {
                double Nx = stretch_dir.x, Ny = stretch_dir.y, Nz = stretch_dir.z;

                double XAx, YAx, ZAx, Ax;
                double XAy, YAy, ZAy, Ay;

                //???????¨¢??¡¤¡§
                if (fabs(Nx) < 1.0 / 64 && fabs(Ny) < 1.0 / 64)
                {
                        Ax = sqrt(pow(Nx, 2) + pow(Nz, 2));

                        XAx = Nz / Ax;
                        YAx = 0;
                        ZAx = -Nx / Ax;

                        Ay = sqrt(pow(Nx, 2) + pow(Ny, 2)
                                  + pow(pow(Nx, 2) + pow(Nz, 2), 2)
                                  + pow(Ny, 2) * pow(Nz, 2));

                        XAy = -Nz * Ny / Ay;
                        YAy = (pow(Nx, 2) + pow(Nz, 2)) / Ay;
                        ZAy = -Ny * Nz / Ay;
                }
                else
                {
                        Ax = sqrt(pow(Nx, 2) + pow(Ny, 2));
                        XAx = -Ny / Ax;
                        YAx = Nx / Ax;;
                        ZAx = 0;

                        Ay = sqrt(pow(Nx, 2) * pow(Nz, 2)
                                  + pow(pow(Nx, 2) + pow(Ny, 2), 2)
                                  + pow(Ny, 2) * pow(Nz, 2));

                        XAy = -Nx * Nz / Ay;
                        YAy = -Ny * Nz / Ay;
                        ZAy = (pow(Ny, 2) + pow(Nz, 2)) / Ay;

                }

                double wx = XAx * x + XAy * y;
                double wy = YAx * x + YAy * y;

                x = wx;
                y = wy;
        }

        bool calcPoint(const dxf::core::point_t &vpt, std::string strOwnerID, std::string layer_name, vector<dxf::core::point_t> &result)
        {
                dxf::core::point_t pt = vpt;

                if (strOwnerID == "")
                {
                        if(pt.color_index == BYLAYER)
                        {
                                pt.color_index = m_mapLayerColorIndex[layer_name];
                        }
                        result = { pt };
                }
                else
                {
                        result = expand_point(pt, strOwnerID, layer_name);
                }

                return result.size() != 0;
        }
private:
        std::vector<dxf::core::point_t> expand_point(dxf::core::point_t pt, std::string strOwnerID, std::string layer_name)
        {
                std::vector<dxf::core::point_t> ret;

                dxf::core::TBlockNode  nodeBlk = m_mapID2Block[strOwnerID];

                pt.x += nodeBlk.m_ptBlockBase.x;
                pt.y += nodeBlk.m_ptBlockBase.y;
                pt.z += nodeBlk.m_ptBlockBase.z;

                std::string block_name = "";
                std::transform(nodeBlk.m_strBlockName.begin(), nodeBlk.m_strBlockName.end(), std::back_inserter(block_name), ::tolower);

                if (pt.color_index == 256 && layer_name != "0")
                {
                        pt.color_index = m_mapLayerColorIndex[layer_name];
                }

                if (pt.color_index == 256 && layer_name == "0")
                {
                        //pt.color_index = nodeBlk.m_iColorIndex;
                }

                if (pt.color_index == 0 && nodeBlk.m_iColorIndex == 256)
                {
                        if(nodeBlk.m_strLayerName != "0")
                                pt.color_index = m_mapLayerColorIndex[nodeBlk.m_strLayerName];
                }

                if (pt.color_index == 0 && nodeBlk.m_iColorIndex != 256)
                {
                        pt.color_index = nodeBlk.m_iColorIndex;
                }

                if (block_name.find("model_space") != std::string::npos)
                {
                        ret.push_back(pt);
                }
                else if (m_mapName2Insert.find(nodeBlk.m_strBlockName) != m_mapName2Insert.end())
                {
                        std::vector<dxf::core::TInsertNode> nodeInserts = m_mapName2Insert[nodeBlk.m_strBlockName];

                        std::for_each(nodeInserts.begin(), nodeInserts.end(), [&](auto nodeInsert) {

                                dxf::core::point_t pt_inner = pt;

                                std::string cur_layer_name = nodeBlk.m_strLayerName;
                                if (nodeInsert.m_strLayerName != "")
                                {
                                        cur_layer_name = nodeInsert.m_strLayerName;
                                }

                                if(pt_inner.color_index == BYLAYER || pt_inner.color_index == BYBLOCK)
                                {
                                        if(nodeInsert.m_iColorIndex == BYLAYER && cur_layer_name != "0")
                                        {
                                                pt_inner.color_index = m_mapLayerColorIndex[cur_layer_name];
                                        }
                                        else if(nodeInsert.m_iColorIndex == BYBLOCK)
                                        {
                                                //nothing
                                        }
                                        else
                                        {
                                                pt_inner.color_index = nodeInsert.m_iColorIndex;
                                        }
                                }

                                transOCStoWCS(nodeInsert.m_vecInsertStretchDirection, pt_inner.x, pt_inner.y);

                                pt_inner.x *= nodeInsert.m_vecInsertScale.x;
                                pt_inner.y *= nodeInsert.m_vecInsertScale.y;
                                pt_inner.z *= nodeInsert.m_vecInsertScale.z;

                                double new_x = pt_inner.x * cos(DEG(nodeInsert.m_dRotateAngle)) - pt_inner.y * sin(DEG(nodeInsert.m_dRotateAngle));
                                double new_y = pt_inner.x * sin(DEG(nodeInsert.m_dRotateAngle)) + pt_inner.y * cos(DEG(nodeInsert.m_dRotateAngle));
                                        
                                pt_inner.angle += DEG(nodeInsert.m_dRotateAngle);

                                pt_inner.x = new_x;
                                pt_inner.y = new_y;

                                pt_inner.x += nodeInsert.m_ptInsertBase.x;
                                pt_inner.y += nodeInsert.m_ptInsertBase.y;
                                pt_inner.z += nodeInsert.m_ptInsertBase.z;

                                std::vector<dxf::core::point_t> pts_inner = expand_point(pt_inner, nodeInsert.m_strParentBlockID, cur_layer_name);
                                std::copy(pts_inner.begin(), pts_inner.end(), std::back_inserter(ret));
                        });
                }
                else
                {
                        ; // do nothing.
                }

                return ret;
        }
private:
        template<typename E>
        void reviseModelSpaceEntityColorIndex(E &entity)
        {
                dxf::core::TBlockNode blkNodeModelSpace =  m_mapID2Block[m_strModelBlockID];
                if(entity.m_iColorIndex == BYLAYER)
                {
                        entity.m_iColorIndex = m_mapLayerColorIndex[blkNodeModelSpace.m_strLayerName];
                }
                if(entity.m_iColorIndex == BYBLOCK)
                {
                        if(blkNodeModelSpace.m_iColorIndex == BYLAYER)
                        {
                                entity.m_iColorIndex = m_mapLayerColorIndex[blkNodeModelSpace.m_strLayerName];
                        }
                        else
                        {
                                entity.m_iColorIndex = blkNodeModelSpace.m_iColorIndex;
                        }
                }
        }
private:
        std::string generateBlockID()
        {

                std::string strBlockID = "";

                do {
                        stringstream sstr;
                        sstr << s_BlockID++;
                        sstr >> strBlockID;
                } while (m_mapID2Block.find(strBlockID) != m_mapID2Block.end());

                return strBlockID;
        }
private:
        std::unique_ptr<DL_Dxf> mDxf;
public:
        std::map<std::string, dxf::core::TBlockNode>   m_mapID2Block;
        std::map<std::string, vector<dxf::core::TInsertNode>>  m_mapName2Insert;
        std::vector<dxf::core::line_t>                 m_vecLines;
        std::vector<dxf::core::point_t>                m_vecPoints;
        std::vector<dxf::core::vertex_t>               m_ptVertexs;
        std::vector<dxf::core::arc_t>                  m_vecArcs;
        std::vector<dxf::core::polyline_t>             m_vecPolylines;
        std::vector<dxf::core::dimAligned_t>           m_vecDimAligneds;
        std::vector<dxf::core::solid_t>                m_vecSolids;
        std::vector<dxf::core::circle_t>               m_vecCircles;
        std::vector<dxf::core::face_3d_t>              m_vecFace3ds;
        std::vector<dxf::core::elipse_t>               m_vecElipses;
        std::vector<dxf::core::text_t>                 m_vecTexts;
        std::vector<dxf::core::mtext_t>                m_vecMTexts;
        std::map<std::string, dxf::core::variable_t>   m_mapVariable;

        std::string m_strModelBlockID;
        std::string m_strCurrentBlockID;

        std::map<std::string, bool>  m_mapLayerVisible;
        std::map<std::string, int>   m_mapLayerColorIndex;
};
}
}

#endif
