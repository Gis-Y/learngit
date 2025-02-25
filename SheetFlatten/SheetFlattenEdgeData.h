#pragma once
#include <TopoDS_Shape.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include<TopoDS_Edge.hxx>
#include <TDF_Label.hxx>
#include <Poly_Array1OfTriangle.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <TDF_LabelSequence.hxx>
#include <BRepTools.hxx>
#include <IMeshTools_Parameters.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <Poly_Triangulation.hxx>
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <TopoDS.hxx>
#include <Poly_PolygonOnTriangulation.hxx>

#include <vector>
//#include<unordered_set>
using namespace std;

class SheetFlattenEdgeData
{
public:
    // Getter and Setter implementations for Edge_3d
    TopoDS_Edge getEdge_3d() const { return  m_Edge_3d; }
    void setEdge_3d(const TopoDS_Edge& edge) { m_Edge_3d = edge; }

    // Getter and Setter implementations for oldEdge_2d
    TopoDS_Edge getOldEdge_2d() { return  m_oldEdge_2d; }
    void setOldEdge_2d(const TopoDS_Edge& edge) { m_oldEdge_2d = edge; }


    // Getter and Setter for vector_newEdge_2d
    TopoDS_Edge getNewEdge_2d()
    {
        if (m_vector_newEdge_2d.size() < 1)
        {
          
            return m_oldEdge_2d;

        }
        return m_vector_newEdge_2d.back();
    }
    bool getVector_newEdge_2d(vector<TopoDS_Edge>& base) 
    {
        if (m_vector_newEdge_2d.size() < 1)
        {
            vector<TopoDS_Edge> aEdges;
            aEdges.emplace_back(m_oldEdge_2d);
            base = aEdges;
            return false;
        }
        base = m_vector_newEdge_2d; 
        return true;
    }
    bool getVector_newEdge_2d(TopoDS_Edge &base) { if (m_vector_newEdge_2d.size() < 1)return false; base = m_vector_newEdge_2d.back(); return true; }
    const vector<TopoDS_Edge>& getVector_newEdge_2d() const { return  m_vector_newEdge_2d; }
    void setVector_newEdge_2d(const vector<TopoDS_Edge>& data) { m_vector_newEdge_2d = data; }

    // Getter and Setter for vector_face
    vector<TopoDS_Face> getVector_face() { return  m_vector_face; }
    const vector<TopoDS_Face>& getVector_face() const { return  m_vector_face; }
    void setVector_face(const vector<TopoDS_Face>& faces) { m_vector_face = faces; }

    // Getter and Setter for vector_sameFace_interseEdge_2d
    bool getVector_sameFace_interseEdge_2d(vector<TopoDS_Edge>& edges) { if (m_vector_sameFace_interseEdge_2d.size() < 1)return false; edges = m_vector_sameFace_interseEdge_2d; return true; }
    void setVector_sameFace_interseEdge_2d(const vector<TopoDS_Edge>& edges) { m_vector_sameFace_interseEdge_2d = edges; }

    // Getter and Setter for vector_interseEdge_2d
    vector<TopoDS_Edge> getVector_interseEdge_2d() { return  m_vector_interseEdge_2d; }
    const vector<TopoDS_Edge>& getVector_interseEdge_2d() const { return  m_vector_interseEdge_2d; }
    void setVector_interseEdge_2d(const vector<TopoDS_Edge>& edges) { m_vector_interseEdge_2d = edges; }

    // Getter and Setter for vector_sameFaceEdge_2d
    vector<TopoDS_Edge> getVector_sameFaceEdge_uninterseEdge_2d() { return  m_vector_sameFaceEdge_uninterseEdge_2d; }
    const vector<TopoDS_Edge>& getVector_sameFaceEdge_uninterseEdge_2d() const { return  m_vector_sameFaceEdge_uninterseEdge_2d; }
    void setVector_sameFaceEdge_uninterseEdge_2d(const vector<TopoDS_Edge>& edges) { m_vector_sameFaceEdge_uninterseEdge_2d = edges; }


    set<TopoDS_Edge> getVector_sameFaceEdge_2d() { return  m_vector_sameFaceEdge_2d; }
    void setVector_sameFaceEdge_2d(const set<TopoDS_Edge>& edges) { m_vector_sameFaceEdge_2d = edges; }


    // Getter and Setter for vector_interseEdge_new2d
    vector<TopoDS_Edge> getVector_interseEdge_new2d() { return  m_vector_interseEdge_new2d; }
    bool getVector_interseEdge_new2d(vector<TopoDS_Edge>& rhs) {
        if (m_vector_interseEdge_new2d.size() < 1)return false; rhs = m_vector_interseEdge_new2d;
        return true;
    }
    void setVector_interseEdge_new2d(const vector<TopoDS_Edge>& edges) { m_vector_interseEdge_new2d = edges; }
    void insertEdgeTo_new2d(const TopoDS_Edge& edge) { m_vector_newEdge_2d.emplace_back(edge); }

    bool getVector_LapelEdges(vector<TopoDS_Edge>& edges) { if (m_LapelEdges_2d.size() < 1)return false; edges = m_LapelEdges_2d; return true; }
    void setVector_LapelEdges(const vector<TopoDS_Edge>& edges) { m_LapelEdges_2d = edges; }


    void getVector_WrapAngleEdges(vector<TopoDS_Edge>& edges) {
        if (m_bAddWrapAngleEdge)getVector_newEdge_2d(edges); else {
            edges = m_vector_wrapAngle;
    } }
    void setVector_WrapAngleEdges(const vector<TopoDS_Edge>& edges) { m_vector_wrapAngle = edges; return; }

    bool isNegativeBend() const { return m_bNegativeBend; }
    void setNegativeBend(bool midleEdge) { m_bNegativeBend = midleEdge; }
    

    bool isTypesetting() const { return m_bTypesetting; }
    void setTypesetting(bool midleEdge) { m_bTypesetting = midleEdge; }

    // Getter and Setter for bOverlapeEdge
    bool isOverlapeEdge() const { return  m_bOverlapeEdge; }
    void setOverlapeEdge(bool overlapeEdge) { m_bOverlapeEdge = overlapeEdge; }

    // Getter and Setter for bSoltEdge
    bool isSoltEdge() const { return  m_bSoltEdge; }
    void setSoltEdge(bool soltEdge) { m_bSoltEdge = soltEdge; }

    // Getter and Setter for bSliceEdge
    bool isSplitEdge() const { return  m_bSplitEdge; }
    void setSplitEdge(bool sliceEdge) { m_bSplitEdge = sliceEdge; }

    // Getter and Setter for angle
    double getAngle() const { return  m_angle; }
    void setAngle(double ang) { m_angle = ang; }

    // Getter and Setter for OutlineEdge
    TopoDS_Edge getOutlineEdge() { return  m_OutlineEdge; }
    void setOutlineEdge(const TopoDS_Edge& edge) { m_OutlineEdge = edge; }


    bool getOverlapeOutlineEdge(TopoDS_Edge& edge) { if (m_OverlapeEdge.IsNull())return false; edge = m_OverlapeEdge; return true; }
    void setOverlapeOutlineEdge(const TopoDS_Edge& edge) { m_OverlapeEdge = edge; }

    // Getter and Setter for overLapEdge_Point
    gp_Pnt getOverLapEdge_Point() const { return  m_overLapEdge_Point; }
    void setOverLapEdge_Point(const gp_Pnt& point) { m_overLapEdge_Point = point; }

    pair<int, int> getSplitIndex()const {return m_SplitIndex;}
    void setSplitIndex(pair<int, int> Index) { m_SplitIndex = Index; }

    bool isAddWrapAngleEdge() { return m_bAddWrapAngleEdge; }
    void setAddWrapAngleEdge(bool WrapAngle) { m_bAddWrapAngleEdge = WrapAngle; }

    bool isWrapAngle() { return m_bWrapAngle; }
    void setWrapAngle(bool WrapAngle) { m_bWrapAngle = WrapAngle; }

    bool isOutline() { return m_bOutline; }
    void setOutline(bool outline) { m_bOutline = outline; }

    bool isProcessWrapAngle() { return m_bProcessWrapAngle; }
    void setProcessWrapAngle(bool data) { m_bProcessWrapAngle = data; }

    void addParallelNumber() { ++m_parallelNumber; }
    int getPatallelNumber() { return m_parallelNumber; }


    void setFirstMoveOverlapeEdge(bool data) { m_bFirstMoveOverlapeEdge = data; }
    bool isFirstMoveOverlapeEdge() { return m_bFirstMoveOverlapeEdge; }

    void setTranslateType(bool data) { m_bTranslateType = data; }
    bool isTranslateType() { return m_bTranslateType; }


    void SetCurveType(CurveType type) {
        m_curveType = type;
    }
    CurveType GetCurveType() const {
        return m_curveType;
    }


    bool isBendEdge() { return m_bBendEdge; }
    void setBendEdge(bool data) { m_bBendEdge = data; }

    bool getNewEdge_2d(TopoDS_Edge& edge) {
        if (m_vector_newEdge_2d.size() > 0)
        {
            edge = m_vector_newEdge_2d.back();
            return true;
        }
        else {
            edge = m_oldEdge_2d;
            return false;
        }
    
    }

    void setFinish(bool rhs)
    {
        m_bFinish = rhs;
    }

    bool isFinish() { return m_bFinish; }
    

    void clear()
    {
        m_vector_newEdge_2d.clear();
        m_vector_face.clear();
        m_vector_interseEdge_new2d.clear();
        m_angle = 0.;
        m_bBendEdge = false;
        m_bOverlapeEdge = false;
        m_bSplitEdge = false;
        m_bSoltEdge = false;
        m_vector_interseEdge_2d.clear();
        m_vector_sameFaceEdge_uninterseEdge_2d.clear();
        m_vector_sameFace_interseEdge_2d.clear();

    }

private:
	TopoDS_Edge m_Edge_3d;
	TopoDS_Edge m_oldEdge_2d;
    vector<TopoDS_Edge> m_LapelEdges_2d;
	vector<TopoDS_Edge> m_vector_newEdge_2d;
	vector<TopoDS_Face> m_vector_face;
	vector<TopoDS_Edge> m_vector_sameFace_interseEdge_2d;//共面且相交
	vector<TopoDS_Edge> m_vector_interseEdge_2d;//只相交，无所谓共面
	vector<TopoDS_Edge> m_vector_sameFaceEdge_uninterseEdge_2d;//共面但不相交
    set<TopoDS_Edge> m_vector_sameFaceEdge_2d;//共面的任意线
	vector<TopoDS_Edge> m_vector_interseEdge_new2d;//共面且相交判断条件在新二维的基础上
    vector<TopoDS_Edge> m_vector_wrapAngle;
	bool m_bOverlapeEdge = false;
    bool m_bFirstMoveOverlapeEdge = false;
    CurveType m_curveType;
    bool m_bWrapAngle = false;
    bool m_bBendEdge = false;
	bool m_bSoltEdge = false;
	bool m_bSplitEdge = false;
    bool m_bFinish = false;
    bool m_bAddWrapAngleEdge = false;
    bool m_bTypesetting = false;
	double m_angle = false;
    bool m_bNegativeBend = false;
    bool m_bOutline = false;
    bool m_bProcessWrapAngle = false;
    int m_parallelNumber = 0;
    pair<int, int> m_SplitIndex;
	TopoDS_Edge m_OutlineEdge;
    TopoDS_Edge m_OverlapeEdge;
	gp_Pnt m_overLapEdge_Point;
    bool m_bTranslateType = false;
public:
    SheetFlattenEdgeData()
		:m_bOverlapeEdge(false), m_bSoltEdge(false), m_angle(0.), m_bFinish(false), m_bBendEdge(false), m_bSplitEdge(false)
        , m_bNegativeBend(false), m_bAddWrapAngleEdge(false), m_bWrapAngle(false)
	{
	}
};
