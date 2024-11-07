#ifndef SHEETFLATTENMOVE_H
#define SHEETFLATTENMOVE_H

#include <GeomAbs_SurfaceType.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <Geom_BSplineSurface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <GeomConvert_SurfToAnaSurf.hxx>
#include <GeomConvert_CurveToAnaCurve.hxx>
#include <Geom_Plane.hxx>
#include <Geom_CylindricalSurface.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <GeomLib.hxx>
#include <GeomAPI.hxx>
#include <TopoDS.hxx>
#include <GeomLib_IsPlanarSurface.hxx>
#include <gp_Vec.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepIntCurveSurface_Inter.hxx>
#include <IntTools_EdgeEdge.hxx>
#include <GeomLProp_SLProps.hxx>



#include "ModelIO/ModelIO.h"
#include "tcl_5.0.6/tree.h"
#include "SheetFlattenBase.h"
#include "SheetFlattenFitting.h"

#include <iostream>
#include <vector>

using namespace std;

typedef struct interseLine_Line
{
	vector<TopoDS_Edge> interseLine;
	vector<TopoDS_Edge> Line;
};

class SheetFalttenMove
{
public:
	SheetFalttenMove();
	~SheetFalttenMove();

private:
	bool getAdjFace(TopoDS_Edge edge, vector<TopoDS_Face>& result);
	bool relationEdge(TopoDS_Edge edge);
	void allReation2dEdge();
	void FindEdgesOnBothSides(const map< TopoDS_Edge, TopoDS_Edge> mapEdges, const TopoDS_Edge& baseEdge,
		std::vector<TopoDS_Edge>& leftEdges, std::vector<TopoDS_Edge>& rightEdges, std::vector<TopoDS_Edge>& colines, std::vector<TopoDS_Edge>& otherlines);
	void calTranslate(const double distance, gp_Trsf& translation, const double& angle);
	TopoDS_Edge calRetractTranslate(const TopoDS_Edge& theEdge, const TopoDS_Face& theFace, const double theDistance, double theAngle);
	TopoDS_Edge TranslateEdge(const TopoDS_Edge& edge, const gp_Trsf& translation);
	bool CalculateAngleBetweenFaces(const vector<TopoDS_Face>& face, double& angle);
	bool CalculateAngleBetweenFaces(const TopoDS_Face& theBaseFace, const TopoDS_Face& theTarFace, double& angle);
	void moveEdge(TopoDS_Edge edge,TopoDS_Compound& aCompound);
	void processOverlap();
	TopoDS_Edge findAdjEdge(TopoDS_Edge baseEdge, gp_Pnt basePoint);
	void generateMidleEdge(const vector<TopoDS_Edge>& theMoveEdge, const gp_Trsf& translation, const gp_Trsf& theTranslation_slot);
	void generateNewEdge(const vector<TopoDS_Edge>& theMoveEdge, const gp_Trsf& translation, bool theMove);
	void generateSoltEdge(const TopoDS_Edge& theEdge);
	void generateOverlapEdge(const TopoDS_Edge& theEdge);
	bool SegmentsOverlap(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, TopoDS_Edge& newEdge1, TopoDS_Edge& newEdge2,
		gp_Pnt& newPoint1, gp_Pnt& newPoint2, gp_Pnt& startPoint1, gp_Pnt& startPoint2, gp_Vec& directionVec1, gp_Vec& directionVec2);
	void processFlotInVec(gp_Vec& normalVec);
	gp_Vec CalMoveVector(const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p);
	void allReation2dNewEdge();
	void findSameEdge(map<TopoDS_Edge, TopoDS_Edge> &result);
	bool TrimEdgesAtIntersection1(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection);
	void openSlot();
	void wrapAngle();
	void makeLapelMap();
	void translateLapel(const double theDistance);
	bool isSameFace(const TopoDS_Edge& theLeftEdge, const TopoDS_Edge& theRightEdge);
	bool AreEdgesSameEndpoints(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2);
	bool IsEdgesAtIntersection(TopoDS_Edge& edge1, TopoDS_Edge& edge2, gp_Pnt& intersection);
	bool findMap_EdgeAndVectorFace_ToFace(const map < TopoDS_Edge, vector<TopoDS_Face>>& theMap, const TopoDS_Edge& theEdge, TopoDS_Face& theFace);
	bool findMap_EdgeAndVEctorFace_ToVectorFace(const map<TopoDS_Edge, vector<TopoDS_Face>>& theMap, const TopoDS_Edge& theEdge, vector<TopoDS_Face>& theFaces);
	bool findMap_EdgeAndEdge_ToEdge(const map<TopoDS_Edge, TopoDS_Edge>& theMapEdges, const TopoDS_Edge& theTarEdge_2d, TopoDS_Edge& theTarEdge_3d);
	bool findMap_EdgeAndVector_ToEdge(const map<TopoDS_Edge, vector<TopoDS_Edge>>& MapEdge, const TopoDS_Edge& theOldEdge, TopoDS_Edge& theNewEdge);
	bool findMap_EdgeAndVector_ToVector(const map<TopoDS_Edge, vector<TopoDS_Edge>>& MapEdge, const TopoDS_Edge& theOldEdge, vector<TopoDS_Edge>& theVector);
	void calTranslateOriention(const TopoDS_Edge& theEdge, const gp_Pnt& thePoint, gp_Trsf& theTranslation, const double theDistance);
public:
	void generate(TopoDS_Compound& aCompound);
	void generateEdge(const TopoDS_Edge& theEdge);
	//void getRefDir(const TopoDS_Edge& edge);
public:
	map<TopoDS_Edge, vector<TopoDS_Face> > m_MapEdgeAdjFaces;
	map<TopoDS_Edge, TopoDS_Edge> m_ThreeToTwoEdge;
	map<TopoDS_Edge, TopoDS_Edge> m_TwoToThreeEdge;
	map<TopoDS_Edge, vector<TopoDS_Edge>>m_oldMapNewEdge;
	map<TopoDS_Edge, TopoDS_Edge>m_newMapOldEdge;
	map<TopoDS_Edge, vector<TopoDS_Edge>>m_MapInterseLines;//旧二维对旧二维
	map<TopoDS_Edge, vector<TopoDS_Edge>>m_MapInterseNewLines;//旧二维对旧二维，只不过判断交线的时候用的是新二维
	map<TopoDS_Edge, interseLine_Line> m_interseLine_LineMap;//三维对旧二维
	map<TopoDS_Edge, vector<TopoDS_Edge>> m_MapLapelEdges;
	set<TopoDS_Edge> m_finishEdge;
	set<TopoDS_Edge> m_overlapEdges;
	map<TopoDS_Edge, gp_Pnt> m_overLapEdge_Point;
	set<TopoDS_Face> m_twoFaces;
	vector<TopoDS_Edge> m_positiveEdges;
	vector<TopoDS_Edge> m_negativeEdges;
	vector<TopoDS_Edge> m_slotEdges_3d;//三维线
	set<TopoDS_Edge> m_slotEdges_2d;
	gp_Dir m_refDir;
	gp_Vec m_baseVec;//移动方向  放
	bool m_isPositiveFold;
};

SheetFalttenMove::SheetFalttenMove()
{
}

SheetFalttenMove::~SheetFalttenMove()
{
}
void SetEdgeColor(const TDF_Label& shapeLabel, const Quantity_Color& color) {
	Handle(XCAFDoc_ColorTool) colorTool = XCAFDoc_DocumentTool::ColorTool(shapeLabel);
	if (!colorTool.IsNull()) {
		// 使用XCAFDoc_ColorType::XCAFDoc_ColorCurves设定边的颜色
		colorTool->SetColor(shapeLabel, color, XCAFDoc_ColorType::XCAFDoc_ColorCurv);
	}
}
void SheetFalttenMove::calTranslateOriention(const TopoDS_Edge& theEdge, const gp_Pnt& thePoint, gp_Trsf& theTranslation ,const double theDistance)
{
	TopoDS_Vertex v1 = TopExp::FirstVertex(theEdge);
	TopoDS_Vertex v2 = TopExp::LastVertex(theEdge);
	gp_Pnt p1 = BRep_Tool::Pnt(v1);
	gp_Pnt p2 = BRep_Tool::Pnt(v2);

	gp_Vec normalVec;
	if (p1.IsEqual(thePoint, 1e-3))
	{
		gp_Vec d1(p1, p2);
		normalVec = d1;
	}
	else if (p2.IsEqual(thePoint, 1e-3))
	{
		gp_Vec d1(p2, p1);
		normalVec = d1;
	}
	else
	{
		return;
	}
	processFlotInVec(normalVec);
	gp_Vec translationVec = normalVec.Normalized() * theDistance;


	theTranslation.SetTranslation(translationVec);  // 设置平移矩阵

	return;
}
//找到线段的另一端
gp_Pnt findOtherPoint(const TopoDS_Edge& theEdge, const gp_Pnt& thePoint)
{
	TopoDS_Vertex v1 = TopExp::FirstVertex(theEdge);
	TopoDS_Vertex v2 = TopExp::LastVertex(theEdge);
	gp_Pnt p1 = BRep_Tool::Pnt(v1);
	gp_Pnt p2 = BRep_Tool::Pnt(v2);

	if (p1.IsEqual(thePoint, 1e-4))
	{
		return p2;
	}
	else
	{
		return p1;
	}
}
void SheetFalttenMove::translateLapel(const double theDistance)
{
	gp_Pnt aInterPoint,aOldInterPoint,aOldPoint1,aOldPoint2;
	TopoDS_Edge aBaseEdge,aLapelEdge1,aLapelEdge2, aLapelNewEdge1, aLapelNewEdge2,aTranslateLaterEdge1,aTranslateLaterEdge2;
	TopoDS_Edge aNewLapelInterEdge1, aNewLapelInterEdge2, aOldLapelInterEdge1, aOldLapelInterEdge2;;
	vector<TopoDS_Edge> aInterLapeEdges1, aInterLapeEdges2;
	gp_Trsf aTranslation;
	for (auto item : m_MapLapelEdges)
	{
		aBaseEdge = item.first;
		aLapelEdge1 = item.second[0];//老二维
		aLapelEdge2 = item.second[1];

		if (m_overlapEdges.find(aLapelEdge1) != m_overlapEdges.end() || m_overlapEdges.find(aLapelEdge2) != m_overlapEdges.end())
		{
			continue;
		}
		IsEdgesAtIntersection(aLapelEdge1, aLapelEdge2, aOldInterPoint);

		aOldPoint1 = findOtherPoint(aLapelEdge1, aOldInterPoint);
		aOldPoint2 = findOtherPoint(aLapelEdge2, aOldInterPoint);
		aOldLapelInterEdge1 = findAdjEdge(aLapelEdge1, aOldPoint1);//老二维
		aOldLapelInterEdge2 = findAdjEdge(aLapelEdge2, aOldPoint2);
		findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, aOldLapelInterEdge1, aNewLapelInterEdge1);//新二维
		findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, aOldLapelInterEdge2, aNewLapelInterEdge2);

		findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, aLapelEdge1, aLapelNewEdge1);//新二维
		findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, aLapelEdge2, aLapelNewEdge2);
		
		IsEdgesAtIntersection(aBaseEdge, item.second[0], aInterPoint);
		calTranslateOriention(aBaseEdge, aInterPoint, aTranslation, theDistance);
		aTranslateLaterEdge1 = TranslateEdge(aLapelNewEdge1, aTranslation);
		aTranslateLaterEdge2 = TranslateEdge(aLapelNewEdge2, aTranslation);


		TrimEdgesAtIntersection1(aTranslateLaterEdge1, aTranslateLaterEdge2, aInterPoint);

		TrimEdgesAtIntersection1(aTranslateLaterEdge1, aNewLapelInterEdge1, aInterPoint);
		TrimEdgesAtIntersection1(aTranslateLaterEdge2, aNewLapelInterEdge2, aInterPoint);


		m_oldMapNewEdge.find(aLapelEdge1)->second.emplace_back(aTranslateLaterEdge1);
		m_oldMapNewEdge.find(aLapelEdge2)->second.emplace_back(aTranslateLaterEdge2);

		m_oldMapNewEdge.find(aOldLapelInterEdge1)->second.emplace_back(aNewLapelInterEdge1);
		m_oldMapNewEdge.find(aOldLapelInterEdge2)->second.emplace_back(aNewLapelInterEdge2);


	}
}
bool SheetFalttenMove::findMap_EdgeAndVector_ToVector(const map<TopoDS_Edge, vector<TopoDS_Edge>>& MapEdge, const TopoDS_Edge& theOldEdge, vector<TopoDS_Edge>& theVector)
{
	if (MapEdge.find(theOldEdge) != MapEdge.end())
	{
		auto iter = MapEdge.find(theOldEdge);
		theVector = iter->second;
		return true;
	}
	return false;
}
bool isSameOrient(const TopoDS_Face& theLeftFace, const TopoDS_Face& theRightFace)
{
	Handle(Geom_Surface) surface1 = BRep_Tool::Surface(theLeftFace);
	Handle(Geom_Surface) surface2 = BRep_Tool::Surface(theRightFace);
	// 假设 surface1 和 surface2 是 Geom_Surface 类型的两个平面
	GeomLProp_SLProps props1(surface1, 0.5, 0.5, 1, Precision::Confusion());
	GeomLProp_SLProps props2(surface2, 0.5, 0.5, 1, Precision::Confusion());

	// 获取两个平面的法线向量
	gp_Dir normal1 = props1.Normal();
	gp_Dir normal2 = props2.Normal();
	if (normal1.IsEqual(normal2, 1e-3))
	{
		return true;
	}
	return false;
}

bool AreAllTrue(const vector<bool>& aResult) {
	return all_of(aResult.begin(), aResult.end(), [](bool val) { return val; });
}
bool SheetFalttenMove::isSameFace(const TopoDS_Edge& theLeftEdge, const TopoDS_Edge& theRightEdge)
{
	vector<TopoDS_Face> aLeftFaces,aRightFaces;
	set<TopoDS_Face> aFinishFaces;
	vector<bool> aResult;
	bool aTempResult = false;
	if (findMap_EdgeAndVEctorFace_ToVectorFace(m_MapEdgeAdjFaces, theLeftEdge, aLeftFaces))
	{
		if (findMap_EdgeAndVEctorFace_ToVectorFace(m_MapEdgeAdjFaces, theRightEdge, aRightFaces))
		{
			for (auto itLeft : aLeftFaces)
			{
				aFinishFaces.insert(itLeft);
				aTempResult = false;
				for (auto itRight : aRightFaces)
				{
					//if (aFinishFaces.find(itRight) == aFinishFaces.end())
					{
						if (itLeft.IsEqual(itRight))//isSameOrient(itLeft, itRight)
						{
							//aTempResult = true;
							return true;
						}
					}
				}
				aResult.emplace_back(aTempResult);
			}
		}
	}
	return false;
	//return AreAllTrue(aResult);
}
bool edgeIsOnFaces(const TopoDS_Edge & theEdge,vector<TopoDS_Face> theFaces)
{
	for (auto elem : theFaces)
	{

		for (TopExp_Explorer explorer(elem, TopAbs_EDGE); explorer.More(); explorer.Next()) {
			const TopoDS_Edge& currentEdge = TopoDS::Edge(explorer.Current());
			if (currentEdge.IsSame(theEdge))
			{
				return true;
			}
		}
	}
	return false;
}
void SheetFalttenMove::makeLapelMap()
{
	TopoDS_Edge aBaseEdge,aBaseEdge3d,aLeftEdge,aRightEdge;
	vector<TopoDS_Edge> aVectorResult;
	vector<TopoDS_Edge> aLapelEdges;
	set<TopoDS_Edge> aFinishEdges;
	for (auto elem : m_oldMapNewEdge)
	{
		if (elem.second.size() >= 3)
		{
			aBaseEdge = elem.first;
			if (!findMap_EdgeAndEdge_ToEdge(m_TwoToThreeEdge, aBaseEdge, aBaseEdge3d))
			{
				return;
			}
			if (findMap_EdgeAndVector_ToVector(m_MapInterseLines, aBaseEdge, aVectorResult))
			{
				for (int i = 0 ;i<aVectorResult.size(); i++)
				{
					findMap_EdgeAndEdge_ToEdge(m_TwoToThreeEdge, aVectorResult[i], aLeftEdge);
					for (int j = 0;j<aVectorResult.size();j++)
					{
						if (j != i && aFinishEdges.find(aVectorResult[i]) == aFinishEdges.end())
						{
							findMap_EdgeAndEdge_ToEdge(m_TwoToThreeEdge, aVectorResult[j], aRightEdge);
							if (AreEdgesSameEndpoints(aLeftEdge, aRightEdge))
							{
								if (!isSameFace(aBaseEdge3d, aRightEdge))
								{
									if (!isSameFace(aBaseEdge3d, aLeftEdge))
									{
										aFinishEdges.insert(aVectorResult[i]);
										aLapelEdges.emplace_back(aVectorResult[i]);
										aLapelEdges.emplace_back(aVectorResult[j]);
										m_MapLapelEdges[aBaseEdge] = aLapelEdges;
										aLapelEdges.clear();
									}
								}
								/*vector<TopoDS_Face> aLeftFaces;
								if (findMap_EdgeAndVEctorFace_ToVectorFace(m_MapEdgeAdjFaces, aBaseEdge3d, aLeftFaces))
								{
									if (!edgeIsOnFaces(aLeftEdge, aLeftFaces))
									{
										if (!edgeIsOnFaces(aRightEdge, aLeftFaces))
										{
											aLapelEdges.emplace_back(it);
											aLapelEdges.emplace_back(it1);
											m_MapLapelEdges[aBaseEdge] = aLapelEdges;
											aLapelEdges.clear();
										}
									}
								}*/
								
							}
						}
					}
				}
			}
			else
			{
				continue;
			}
		}
	}
}
void SheetFalttenMove::openSlot()
{
	TopoDS_Edge aEdge_2d;
	for (auto elem : m_slotEdges_3d)
	{
		if (findMap_EdgeAndEdge_ToEdge(m_ThreeToTwoEdge, elem, aEdge_2d))
		{
			m_slotEdges_2d.insert(aEdge_2d);
		}
	}
}

//求交点坐标
bool SheetFalttenMove::IsEdgesAtIntersection(TopoDS_Edge& edge1, TopoDS_Edge& edge2, gp_Pnt& intersection) {
	// 获取第一条线段的起点和终点
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// 获取第二条线段的起点和终点
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);
	// 计算两条线段的方向向量
	gp_Vec d1(p1_1, p1_2); // 第一条线段的方向向量
	gp_Vec d2(p2_1, p2_2); // 第二条线段的方向向量

	gp_Vec crossProduct = d1.Crossed(d2);

	// 判断是否平行
	if (crossProduct.SquareMagnitude() < 1e-12) {
		return false; // 平行，无交点
	}
	gp_Vec p1p3(p1_1, p2_1);
	// 计算 t 和 u
	double t = (p1p3.Crossed(d2)).Dot(crossProduct) / crossProduct.SquareMagnitude();
	double u = (p1p3.Crossed(d1)).Dot(crossProduct) / crossProduct.SquareMagnitude();

	// 计算交点坐标
	intersection = p1_1.Translated(t * d1);
	return true;
}
void SheetFalttenMove::processFlotInVec(gp_Vec &normalVec)
{
	if (normalVec.X() != 0. && (normalVec.Y() / normalVec.X() > 500 || normalVec.Y() / normalVec.X() < -500))
	{
		normalVec.SetX(0.);
	}
	if (normalVec.Y() != 0. && (normalVec.X() / normalVec.Y() > 500 || normalVec.X() / normalVec.Y() < -500))
	{
		normalVec.SetY(0.);
	}
}
bool isOverlap(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2)
{
	gp_Pnt basePoint;
	// 获取线段1的端点
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// 获取线段2的端点
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);

	// 计算方向向量
	gp_Vec d1(p1_1, p1_2);
	gp_Vec d2(p2_1, p2_2);

	// 判断方向向量是否共线 (叉积是否接近零)
	gp_Vec crossProduct = d1.Crossed(d2);
	if (crossProduct.SquareMagnitude() > 1e-6) {
		// 如果叉积不为零，说明不共线
		return false;
	}

	// 检查是否在同一直线上，可以比较两个点的方向向量
	gp_Vec dir1(p1_1, p2_1);
	gp_Vec dir2(p1_1, p2_2);
	if (std::abs(dir1.Crossed(d1).SquareMagnitude()) > 1e-6) {
		return false; // 不在同一直线
	}
	if (d1.Dot(d2) > 0)
	{
		if (d1.Dot(dir1) > 0)
		{
			basePoint = p1_1;
		}
		else
		{
			basePoint = p2_1;
		}
	}
	else {
		if (d1.Dot(dir2) > 0)
		{
			basePoint = p1_1;
		}
		else
		{
			basePoint = p2_2;
		}
	}
	// 将四个端点存储在数组中
	std::vector<gp_Pnt> points = { p1_1, p1_2, p2_1, p2_2 };
	gp_Pnt aPoint1;
	bool aIsInter = false;
	// 按照在线段方向上投影的距离排序
	std::sort(points.begin(), points.end(), [&](const gp_Pnt& a, const gp_Pnt& b) {
		return a.Distance(basePoint) < b.Distance(basePoint);
		});
	if (basePoint.IsEqual(p1_1, 1e-6))
	{
		if (!points[1].IsEqual(p1_2, 1e-6))
		{
			aIsInter = true;
		}
	}
	if (basePoint.IsEqual(p2_1, 1e-6))
	{
		if (!points[1].IsEqual(p2_2, 1e-6))
		{
			aIsInter = true;
		}
	}
	if (basePoint.IsEqual(p2_2, 1e-6))
	{
		if (!points[1].IsEqual(p2_1, 1e-6))
		{
			aIsInter = true;
		}
	}
	if (!points[1].IsEqual(points[2], 1e-6) && aIsInter)
	{
		return true;
	}
	return false;
}
bool SheetFalttenMove::SegmentsOverlap(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, TopoDS_Edge& newEdge1, TopoDS_Edge& newEdge2,
 gp_Pnt &newPoint1,gp_Pnt &newPoint2,gp_Pnt &startPoint1,gp_Pnt &startPoint2, gp_Vec &directionVec1, gp_Vec& directionVec2) {
	gp_Pnt basePoint;
	// 获取线段1的端点
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// 获取线段2的端点
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);

	// 计算方向向量
	gp_Vec d1(p1_1, p1_2);
	gp_Vec d2(p2_1, p2_2);

	// 判断方向向量是否共线 (叉积是否接近零)
	gp_Vec crossProduct = d1.Crossed(d2);
	if (crossProduct.SquareMagnitude() > 1e-6) {
		// 如果叉积不为零，说明不共线
		return false;
	}

	// 检查是否在同一直线上，可以比较两个点的方向向量
	gp_Vec dir1(p1_1, p2_1);
	gp_Vec dir2(p1_1, p2_2);
	if (std::abs(dir1.Crossed(d1).SquareMagnitude()) > 1e-3) {
		return false; // 不在同一直线
	}
	if (d1.Dot(d2) > 0)
	{
		if (d1.Dot(dir1) > 0)
		{
			basePoint = p1_1;
		}
		else
		{
			basePoint = p2_1;
		}
	}
	else {
		if (d1.Dot(dir2) > 0)
		{
			basePoint = p1_1;
		}
		else
		{
			basePoint = p2_2;
		}
	}
	// 将四个端点存储在数组中
	std::vector<gp_Pnt> points = { p1_1, p1_2, p2_1, p2_2 };
	gp_Pnt aPoint1;
	bool aIsInter = false;
	// 按照在线段方向上投影的距离排序
	std::sort(points.begin(), points.end(), [&](const gp_Pnt& a, const gp_Pnt& b) {
		return a.Distance(basePoint) < b.Distance(basePoint);
		});
	if (basePoint.IsEqual(p1_1, 1e-6))
	{
		if (!points[1].IsEqual(p1_2, 1e-6))
		{
			aIsInter = true;
		}
	}
	if (basePoint.IsEqual(p2_1, 1e-6))
	{
		if (!points[1].IsEqual(p2_2, 1e-6))
		{
			aIsInter = true;
		}
	}
	if (basePoint.IsEqual(p2_2, 1e-6))
	{
		if (!points[1].IsEqual(p2_1, 1e-6))
		{
			aIsInter = true;
		}
	}
	if (!points[1].IsEqual(points[2], 1e-6) && aIsInter)
	{
		TopoDS_Edge tempEdge1, teedge2;
		if (points[0].IsEqual(p1_1, 1e-6) || points[0].IsEqual(p1_2, 1e-6))
		{
			newPoint1 = points[1];
			newPoint2 = points[2];
			startPoint1 = points[3];
			startPoint2 = points[0];
			//newEdge1 = BRepBuilderAPI_MakeEdge(points[0], points[2]);
			newEdge1 = BRepBuilderAPI_MakeEdge(points[0], points[1]);
			// 创建从 p1 到 p2 的向量
			gp_Vec aDirectionVec(points[1], points[3]);
			processFlotInVec(aDirectionVec);
			aDirectionVec.Normalize();
			directionVec1 = aDirectionVec;

			//newEdge2 = BRepBuilderAPI_MakeEdge(points[1], points[3]);
			newEdge2 = BRepBuilderAPI_MakeEdge(points[2], points[3]);
			gp_Vec aDirectionVec1(points[2], points[0]);
			processFlotInVec(aDirectionVec1);
			aDirectionVec1.Normalize();
			directionVec2 = aDirectionVec1;
		}
		else
		{
			newPoint2 = points[1];
			newPoint1 = points[2];
			startPoint2 = points[3];
			startPoint1 = points[0];
			//newEdge2 = BRepBuilderAPI_MakeEdge(points[0], points[2]);
			newEdge2 = BRepBuilderAPI_MakeEdge(points[0], points[1]);
			gp_Vec aDirectionVec1(points[1], points[3]);
			processFlotInVec(aDirectionVec1);
			aDirectionVec1.Normalize();
			directionVec2 = aDirectionVec1;


			newEdge1 = BRepBuilderAPI_MakeEdge(points[2], points[3]);
			//newEdge1 = BRepBuilderAPI_MakeEdge(points[1], points[3]);
			gp_Vec aDirectionVec(points[2], points[0]);
			processFlotInVec(aDirectionVec);
			aDirectionVec.Normalize();
			directionVec1 = aDirectionVec;
		}
		return true;
	}
	return false;
}

//
gp_Pnt ProjectPointOntoEdge(const gp_Pnt& point, const TopoDS_Edge& edge) {
	// 获取边的两个端点
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge);
	TopoDS_Vertex v2 = TopExp::LastVertex(edge);

	// 提取端点坐标
	gp_Pnt p1 = BRep_Tool::Pnt(v1);
	gp_Pnt p2 = BRep_Tool::Pnt(v2);

	// 计算边的方向向量
	gp_Vec d(p1, p2);

	// 计算从 p1 到 point 的向量
	gp_Vec pointVector(p1, point);

	// 计算点到边的投影系数
	double t = pointVector.Dot(d) / d.SquareMagnitude();

	// 限制 t 的范围在 [0, 1] 内，确保投影点在边上
	if (t < 0) {
		return p1; // 返回端点 p1
	}
	else if (t > 1) {
		return p2; // 返回端点 p2
	}

	// 计算投影点
	gp_Pnt projection = p1.Translated(d.Multiplied(t));
	return projection;
}

//通过交点找交边
TopoDS_Edge SheetFalttenMove::findAdjEdge(const TopoDS_Edge baseEdge, const gp_Pnt basePoint)
{
	if (m_MapInterseLines.find(baseEdge) != m_MapInterseLines.end())
	{
		for (auto it : m_MapInterseLines.find(baseEdge)->second)
		{
			// 获取边的两个端点
			TopoDS_Vertex v1 = TopExp::FirstVertex(it);
			TopoDS_Vertex v2 = TopExp::LastVertex(it);

			// 提取端点坐标
			gp_Pnt p1 = BRep_Tool::Pnt(v1);
			gp_Pnt p2 = BRep_Tool::Pnt(v2);
			if (basePoint.IsEqual(p1, 1e-4) || basePoint.IsEqual(p2, 1e-4))
			{
				TopoDS_Edge edge = it;
				return edge;
			}
		}
	}
}
void updateData(map<TopoDS_Edge, TopoDS_Edge>& MapEdge, const TopoDS_Edge& theOldEdge, const TopoDS_Edge& theNewEdge)
{
	if (MapEdge.find(theOldEdge) != MapEdge.end())
	{
		auto iter = MapEdge.find(theOldEdge);
		MapEdge[theNewEdge] = iter->second;
	}
	return;
}
bool SheetFalttenMove::findMap_EdgeAndVector_ToEdge(const map<TopoDS_Edge, vector<TopoDS_Edge>>& MapEdge, const TopoDS_Edge& theOldEdge, TopoDS_Edge& theNewEdge)
{
	if (MapEdge.find(theOldEdge) != MapEdge.end())
	{
		auto iter = MapEdge.find(theOldEdge);
		theNewEdge = iter->second[0];
		return true;
	}
	return false;
}


bool SheetFalttenMove::findMap_EdgeAndEdge_ToEdge(const map<TopoDS_Edge, TopoDS_Edge> & theMapEdges, const TopoDS_Edge & theTarEdge_2d, TopoDS_Edge & theTarEdge_3d)
{
	if (theMapEdges.find(theTarEdge_2d) != theMapEdges.end())
	{
		theTarEdge_3d = theMapEdges.find(theTarEdge_2d)->second;
		return true;
	}
	else
	{
		return false;
	}
}

bool SheetFalttenMove::findMap_EdgeAndVectorFace_ToFace(const map < TopoDS_Edge, vector<TopoDS_Face>>& theMap, const TopoDS_Edge& theEdge, TopoDS_Face& theFace)
{
	if (theMap.find(theEdge) != theMap.end())
	{
		theFace = theMap.find(theEdge)->second[0];
		return true;
	}
	else
	{
		return false;
	}

}

bool SheetFalttenMove::findMap_EdgeAndVEctorFace_ToVectorFace(const map<TopoDS_Edge, vector<TopoDS_Face>>& theMap, const TopoDS_Edge& theEdge, vector<TopoDS_Face>& theFaces)
{
	if (theMap.find(theEdge) != theMap.end())
	{
		theFaces = theMap.find(theEdge)->second;
		return true;
	}
	return false;
}
// 函数：求点到直线的垂足交点
gp_Pnt GetPerpendicularFootPoint(const gp_Pnt& point, const TopoDS_Edge& edge)
{
	// 获取直线的两个端点
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge);
	TopoDS_Vertex v2 = TopExp::LastVertex(edge);
	gp_Pnt p1 = BRep_Tool::Pnt(v1);
	gp_Pnt p2 = BRep_Tool::Pnt(v2);

	// 计算直线的方向向量
	gp_Vec lineVec(p1, p2);

	// 计算点 P 到直线上的投影
	gp_Vec p1ToPoint(p1, point);  // 计算 P1 到点的向量
	double projectionLength = p1ToPoint.Dot(lineVec) / lineVec.SquareMagnitude();

	// 根据投影长度计算垂足点坐标
	gp_Vec projectionVec = lineVec.Scaled(projectionLength);  // 计算投影向量
	gp_Pnt intersectionPoint = p1.Translated(projectionVec);  // 投影点就是垂足

	return intersectionPoint;
}
void SheetFalttenMove::processOverlap()
{
	TopoDS_Edge tempEdge1, tempEdge2,oldTarEdge1_2d,oldTarEdge2_2d, newTarEdge1, newTarEdge2;
	gp_Vec aDirvec1, aDirvec2;
	gp_Pnt aInterPoint1, aInterPoint2,aStartPoint1,aStartPoint2,aNewPoint1,aNewPoint2;
	for (auto it : m_ThreeToTwoEdge)
	{
		for (auto item : m_ThreeToTwoEdge)
		{
			TopoDS_Edge new1, new2;
			if (!findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, it.second, new1))
			{
				new1 = it.second;
			}
			if (!findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, item.second, new2))
			{
				new2 = item.second;
			}
			if (new1 != new2)
			{

				if (SegmentsOverlap(new1, new2, tempEdge1, tempEdge2, aInterPoint1, aInterPoint2,  aStartPoint1, aStartPoint2 ,aDirvec1, aDirvec2))
				{
					oldTarEdge1_2d = findAdjEdge(it.second, aInterPoint2);//交线里找
					oldTarEdge2_2d = findAdjEdge(item.second, aInterPoint1);
					m_overlapEdges.insert(oldTarEdge1_2d);
					m_overlapEdges.insert(oldTarEdge2_2d);

					gp_Pnt aPoint1,aPoint2;
					IsEdgesAtIntersection(oldTarEdge1_2d,oldTarEdge2_2d, aPoint1);//找到交点
					aPoint2 = aPoint1;
					// 缩放单位向量，根据给定的距离生成位移向量
					
					double distance = 3.;
					gp_Vec displacement1 = aDirvec1.Scaled(distance);	
					// 将初始点 p1 沿着方向向量移动，生成新的点
					aPoint1 = aPoint1.Translated(displacement1);//移动后的交点
					gp_Pnt vecticalPoint1 = GetPerpendicularFootPoint(aPoint1, tempEdge1);//垂线交点
					newTarEdge1 = BRepBuilderAPI_MakeEdge(aPoint1, vecticalPoint1);
					tempEdge1 = BRepBuilderAPI_MakeEdge(aStartPoint1, vecticalPoint1);


					gp_Vec displacement2 = aDirvec2.Scaled(distance);
					// 将初始点 p1 沿着方向向量移动，生成新的点
					aPoint2 = aPoint2.Translated(displacement2);
					gp_Pnt vecticalPoint2 = GetPerpendicularFootPoint(aPoint2, tempEdge2);
					newTarEdge2 = BRepBuilderAPI_MakeEdge(aPoint2, vecticalPoint2);
					tempEdge2 = BRepBuilderAPI_MakeEdge(vecticalPoint2, aStartPoint2);
					
					//通过2d线找到3d线，找到面，找到共线的线
					
					
					/*if (_2dTo3dEdge(m_TwoToThreeEdge, oldTarEdge1_2d,oldTarEdge1_3d))
					{
						TopoDS_Face aFace1, aFace2;
						if (_3dEdgeToFace(m_MapEdgeAdjFaces, oldTarEdge1_2d, aFace1))
						{

						}
					}
					relationEdge(it.first);
					relationEdge(item.first);
					if (m_interseLine_LineMap.find(it.first) != m_interseLine_LineMap.end())
					{
						for (auto elem : m_interseLine_LineMap.find(it.first)->second.Line)
						{
							gp_Pnt tarPoint = ProjectPointOntoEdge(aInterPoint1, elem);
							newTarEdge1 = BRepBuilderAPI_MakeEdge(aInterPoint1, tarPoint);
						}
					}
					if (m_interseLine_LineMap.find(item.first) != m_interseLine_LineMap.end())
					{
						for (auto elem : m_interseLine_LineMap.find(item.first)->second.Line)
						{
							gp_Pnt tarPoint = ProjectPointOntoEdge(aInterPoint2, elem);
							newTarEdge2 = BRepBuilderAPI_MakeEdge(aInterPoint2, tarPoint);
						}
					}*/
					
					
					/*if (m_TwoToThreeEdge.find(it.second[0]) != m_TwoToThreeEdge.end())
					{
						auto iter = m_TwoToThreeEdge.find(it.second[0]);
						m_TwoToThreeEdge[tempEdge1] = iter->second;
					}
					if (m_TwoToThreeEdge.find(item.second[0]) != m_TwoToThreeEdge.end())
					{
						auto iter = m_TwoToThreeEdge.find(item.second[0]);
						m_TwoToThreeEdge[tempEdge2] = iter->second;
					}
					it.second[0] = tempEdge1;
					item.second[0] = tempEdge2;
					updateData(m_TwoToThreeEdge, oldTarEdge1, newTarEdge1);
					updateData(m_TwoToThreeEdge, oldTarEdge2, newTarEdge2);*/
					vector<TopoDS_Edge> aEdges;
					aEdges.emplace_back(tempEdge2);
					m_oldMapNewEdge[it.second] = aEdges;
					aEdges.clear();


					aEdges.emplace_back(tempEdge1);
					m_oldMapNewEdge[item.second] = aEdges;
					aEdges.clear();

					aEdges.emplace_back(newTarEdge1);
					m_oldMapNewEdge[oldTarEdge1_2d] = aEdges;
					aEdges.clear();

					aEdges.emplace_back(newTarEdge2);
					m_oldMapNewEdge[oldTarEdge2_2d] = aEdges;
					aEdges.clear();


					TopoDS_Compound aCompound;
					BRep_Builder aBuilder;
					aBuilder.MakeCompound(aCompound);
					for (auto it : m_ThreeToTwoEdge)
					{
						TopoDS_Edge edge;
						vector<TopoDS_Edge> aEdges;
						edge = it.second;
						if (m_oldMapNewEdge.find(edge) != m_oldMapNewEdge.end())
						{
							aEdges = m_oldMapNewEdge.find(edge)->second;
							for (auto elem : aEdges)
							{
								aBuilder.Add(aCompound, elem);
							}
						}
						else
						{
							aBuilder.Add(aCompound, edge);
						}
					}
					FlattenFace::BuildDocStd(aCompound);

				}
			}
		}
	}
}
/*
void SheetFalttenMove::getRefDir(const TopoDS_Edge & edge)
{
	if (m_MapEdgeAdjFaces.find(edge) != m_MapEdgeAdjFaces.end())
	{
		TopoDS_Face face = m_MapEdgeAdjFaces.find(edge)->second[0];
		Handle(Geom_Surface) surface1 = BRep_Tool::Surface(face);
		face = m_MapEdgeAdjFaces.find(edge)->second[1];
		Handle(Geom_Surface) surface2 = BRep_Tool::Surface(face);
		// 假设 surface1 和 surface2 是 Geom_Surface 类型的两个平面
		GeomLProp_SLProps props1(surface1, 0.5, 0.5, 1, Precision::Confusion());
		GeomLProp_SLProps props2(surface2, 0.5, 0.5, 1, Precision::Confusion());

		// 获取两个平面的法线向量
		gp_Dir normal1 = props1.Normal();
		gp_Dir normal2 = props2.Normal();

		// 假设 referenceVec 是平面交线的方向向量
		refDir = normal1 ^ normal2;
	}
}
*/
int GetPointSide1(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point) {
	gp_Vec baseVec(baseStart, baseEnd);  // 基准边的方向向量
	gp_Vec vecToPoint(baseStart, point); // 从基准边起点指向当前点的向量

	// 计算叉积的Z分量，判断点在基准边的哪一侧
	gp_Vec crossProduct = baseVec ^ vecToPoint;

	if (crossProduct.Z() > 1e-3) {
		return 1;  // 点在基准边的一侧
	}
	else if (crossProduct.Z() < -1e-3) {
		return -1; // 点在基准边的另一侧
	}
	return 0;  // 点在线上
}
int GetPointSide(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point) {
	gp_Vec baseVec(baseStart, baseEnd);  // 基准边的方向向量
	gp_Vec vecToPoint(baseStart, point); // 从基准边起点指向当前点的向量

	// 计算叉积的Z分量，判断点在基准边的哪一侧
	gp_Vec crossProduct = baseVec ^ vecToPoint;

	if (crossProduct.Z() > 5.) {
		return 1;  // 点在基准边的一侧
	}
	else if (crossProduct.Z() < -5.) {
		return -1; // 点在基准边的另一侧
	}
	return 0;  // 点在线上
}

//计算移动方向
gp_Vec SheetFalttenMove::CalMoveVector(const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p) {
	gp_Vec lineVec(p1, p2); // 从 p1 到 p2 的向量

	gp_Vec perpendicularVec = gp_Vec(0, 0, 1) ^ lineVec; // 计算垂直于lineVec的向量，这里假设lineVec不在Z轴方向

	gp_Vec pointVec(p2, p); // 从 p2 到 p 的向量

	if (pointVec.Dot(perpendicularVec) < 0) {
		perpendicularVec *= -1; // 确保垂直向量指向点 p
	}
	processFlotInVec(perpendicularVec);
	perpendicularVec.Normalize();

	return perpendicularVec; // 返回单位向量
}

// 函数：找到位于基准线两侧的所有边
void SheetFalttenMove::FindEdgesOnBothSides(const map< TopoDS_Edge, TopoDS_Edge> mapEdges, const TopoDS_Edge& baseEdge,
	std::vector<TopoDS_Edge>& leftEdges, std::vector<TopoDS_Edge>& rightEdges, std::vector<TopoDS_Edge>& colines, std::vector<TopoDS_Edge>& otherlines) {
	// 获取基准边的起点和终点
	TopoDS_Edge aBaseEdge = baseEdge;//旧二维
	if (m_oldMapNewEdge.find(aBaseEdge) != m_oldMapNewEdge.end())
	{
		aBaseEdge = m_oldMapNewEdge.find(aBaseEdge)->second[0];
	}
	TopoDS_Vertex v1 = TopExp::FirstVertex(aBaseEdge);  // 获取起点
	TopoDS_Vertex v2 = TopExp::LastVertex(aBaseEdge);   // 获取终点

	gp_Pnt baseStart = BRep_Tool::Pnt(v1);  // 获取起点坐标
	gp_Pnt baseEnd = BRep_Tool::Pnt(v2);

	// 遍历模型中的所有边
	for (auto item : mapEdges) {
		TopoDS_Edge currentEdge = item.second;//旧二维
		if (m_oldMapNewEdge.find(currentEdge) != m_oldMapNewEdge.end())
		{
			currentEdge = m_oldMapNewEdge.find(currentEdge)->second[0];
		}
		TopoDS_Vertex w1 = TopExp::FirstVertex(currentEdge);  // 获取起点
		TopoDS_Vertex w2 = TopExp::LastVertex(currentEdge);
		gp_Pnt startPnt = BRep_Tool::Pnt(w1);
		gp_Pnt endPnt = BRep_Tool::Pnt(w2);

		// 判断当前边的两个端点分别在基准边的哪一侧
		int sideStart = GetPointSide1(baseStart, baseEnd, startPnt);
		int sideEnd = GetPointSide1(baseStart, baseEnd, endPnt);
		if (sideStart > 0 && sideEnd > 0)
		{
			gp_Vec vec1 = CalMoveVector(baseStart, baseEnd, startPnt);
			gp_Vec vec2 = CalMoveVector(baseStart, baseEnd, endPnt);
			if (vec1.IsEqual(vec2,1e-4,1e-4))
			{
				m_baseVec = CalMoveVector(baseStart, baseEnd, startPnt);
			}
		}
		sideStart = GetPointSide(baseStart, baseEnd, startPnt);
		sideEnd = GetPointSide(baseStart, baseEnd, endPnt);
		// 如果两个端点都在同一侧
		if ((sideStart >= 0 && sideEnd > 0) || ((sideStart > 0 && sideEnd >= 0))) {
			rightEdges.push_back(item.second);  // 在右侧
		}
		else if ((sideStart <= 0 && sideEnd < 0) || ((sideStart < 0 && sideEnd <= 0))) {
			leftEdges.push_back(item.second); // 在左侧
		}
		else if (sideStart == 0 && sideEnd == 0)
		{
			colines.push_back(item.second);
		}
		else {
			otherlines.push_back(item.second);
		}
	}
}


TopoDS_Edge SheetFalttenMove::TranslateEdge(const TopoDS_Edge& edge, const gp_Trsf& translation)
{
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge);  // 获取起点
	TopoDS_Vertex v2 = TopExp::LastVertex(edge);   // 获取终点
	gp_Vec translationVec = translation.TranslationPart();
	gp_Pnt p1 = BRep_Tool::Pnt(v1);  // 获取起点坐标
	gp_Pnt p2 = BRep_Tool::Pnt(v2);
	// 5. 对边的两个端点应用平移
	p1.Transform(translation);  // 移动起点
	p2.Transform(translation);  // 移动终点

	// 6. 使用移动后的点构造新的边
	TopoDS_Edge newEdge = BRepBuilderAPI_MakeEdge(p1, p2);

	return newEdge;
}
//包角时朝着向量方向移动边,裁剪边
TopoDS_Edge SheetFalttenMove::calRetractTranslate(const TopoDS_Edge& theEdge,const TopoDS_Face & theFace, const double theDistance ,double theAngle) {
	// 1. 获取边的两个端点
	gp_Trsf translation;
	TopoDS_Edge a2dBaseEdge,a2dTarEdge,a2dNewEdge,a2dInterEdge;
	TopoDS_Edge newEdge;
	gp_Pnt p1, p2,intersection;
	bool isReduct = true;
	relationEdge(theEdge);
	if (findMap_EdgeAndEdge_ToEdge(m_ThreeToTwoEdge, theEdge, a2dBaseEdge))
	{
		if (findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, a2dBaseEdge, a2dNewEdge))
		{
			TopoDS_Vertex w1 = TopExp::FirstVertex(a2dNewEdge);  // 获取起点
			TopoDS_Vertex w2 = TopExp::LastVertex(a2dNewEdge);   // 获取终点

			p1 = BRep_Tool::Pnt(w1);  // 获取起点坐标
			p2 = BRep_Tool::Pnt(w2);
		}
		TopoDS_Vertex v1 = TopExp::FirstVertex(a2dBaseEdge);  // 获取起点
		TopoDS_Vertex v2 = TopExp::LastVertex(a2dBaseEdge);   // 获取终点

		gp_Pnt baseStart = BRep_Tool::Pnt(v1);  // 获取起点坐标
		gp_Pnt baseEnd = BRep_Tool::Pnt(v2);  // 获取终点坐标


		gp_Vec normalVec;


		for (TopExp_Explorer explorer(theFace, TopAbs_EDGE); explorer.More(); explorer.Next()) {
			const TopoDS_Edge& currentEdge = TopoDS::Edge(explorer.Current());
			if (!findMap_EdgeAndEdge_ToEdge(m_ThreeToTwoEdge, currentEdge, a2dTarEdge))
			{
				continue;
			}
			TopoDS_Vertex w1 = TopExp::FirstVertex(a2dTarEdge);  // 获取起点
			TopoDS_Vertex w2 = TopExp::LastVertex(a2dTarEdge);
			gp_Pnt startPnt = BRep_Tool::Pnt(w1);
			gp_Pnt endPnt = BRep_Tool::Pnt(w2);

			// 判断当前边的两个端点分别在基准边的哪一侧
			int sideStart = GetPointSide1(baseStart, baseEnd, startPnt);
			int sideEnd = GetPointSide1(baseStart, baseEnd, endPnt);
			if (sideStart != 0)
			{
				normalVec = CalMoveVector(baseStart, baseEnd, startPnt);
			}
			if (sideEnd != 0)
			{
				normalVec = CalMoveVector(baseStart, baseEnd, endPnt);
			}

		}

		if (theAngle < M_PI)
		{
			normalVec = -normalVec;
			isReduct = false;
		}
		//标记
		processFlotInVec(normalVec);
		// 4. 创建平移变换矩阵
		//gp_Vec translationVec = normalVec.Normalized() * distance;  // 法向量单位化并乘以平移距离
		gp_Vec translationVec = normalVec.Normalized() * theDistance;


		translation.SetTranslation(translationVec);  // 设置平移矩阵

		// 5. 对边的两个端点应用平移
		p1.Transform(translation);  // 移动起点
		p2.Transform(translation);  // 移动终点

		// 6. 使用移动后的点构造新的边
		newEdge = BRepBuilderAPI_MakeEdge(p1, p2);
	}
	if (fabs(M_PI - theAngle) <= 1e-6)
	{
		return newEdge;
	}
	if (theAngle >= M_PI / 2 && theAngle <= M_PI)
	{
		return newEdge;
	}
	if (m_interseLine_LineMap.find(theEdge) != m_interseLine_LineMap.end())
	{
		for (auto elem : m_interseLine_LineMap.find(theEdge)->second.interseLine)
		{
			if (findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, elem, a2dInterEdge))
			{
				if (!isReduct)
				{
					TrimEdgesAtIntersection1(newEdge, a2dInterEdge, intersection);
					m_oldMapNewEdge[elem][0] = a2dInterEdge;
				}
				//TrimEdgesAtIntersection1(newEdge, a2dInterEdge, intersection);
				//m_oldMapNewEdge[elem][0] = a2dInterEdge;
			}
		}
	}
	m_oldMapNewEdge[a2dBaseEdge].emplace_back(newEdge);
	return newEdge;  // 返回新的边
}
//朝着向量方向移动边
void SheetFalttenMove::calTranslate(const double distance, gp_Trsf& translation ,const double & angle) {
	gp_Vec vec = m_baseVec;
	//gp_Vec translationVec = normalVec.Normalized() * distance;  // 法向量单位化并乘以平移距离
	if (angle > M_PI)
	{
		vec = -m_baseVec;
	}
	gp_Vec translationVec = vec.Normalized() * distance;
	translation.SetTranslation(translationVec);  // 设置平移矩阵
	return; // 返回新的边
}

//计算中间线
TopoDS_Edge midleLine(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2)
{
	// 获取第一条边的两个端点
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge1);  // 获取起点
	TopoDS_Vertex v2 = TopExp::LastVertex(edge1);
	gp_Pnt startPnt1 = BRep_Tool::Pnt(v1);
	gp_Pnt endPnt1 = BRep_Tool::Pnt(v2);

	// 获取第二条边的两个端点
	v1 = TopExp::FirstVertex(edge2);
	v2 = TopExp::LastVertex(edge2);
	gp_Pnt startPnt2 = BRep_Tool::Pnt(v1);
	gp_Pnt endPnt2 = BRep_Tool::Pnt(v2);

	// 计算起点的中点
	gp_Pnt midStart((startPnt1.X() + startPnt2.X()) / 2.0,
		(startPnt1.Y() + startPnt2.Y()) / 2.0,
		(startPnt1.Z() + startPnt2.Z()) / 2.0);

	// 计算终点的中点
	gp_Pnt midEnd((endPnt1.X() + endPnt2.X()) / 2.0,
		(endPnt1.Y() + endPnt2.Y()) / 2.0,
		(endPnt1.Z() + endPnt2.Z()) / 2.0);

	// 构建中间线
	TopoDS_Edge middleEdge = BRepBuilderAPI_MakeEdge(midStart, midEnd);

	return middleEdge;

}
//延长到交点，交点在任意处
bool SheetFalttenMove::TrimEdgesAtIntersection1(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection) {
	// 获取第一条线段的起点和终点
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(theEdge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(theEdge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// 获取第二条线段的起点和终点
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(theEdge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(theEdge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);
	// 计算两条线段的方向向量
	gp_Vec d1(p1_1, p1_2); // 第一条线段的方向向量
	gp_Vec d2(p2_1, p2_2); // 第二条线段的方向向量

	gp_Vec crossProduct = d1.Crossed(d2);

	// 判断是否平行
	if (crossProduct.SquareMagnitude() < 1e-12) {
		return false; // 平行，无交点
	}
	gp_Vec p1p3(p1_1, p2_1);
	// 计算 t 和 u
	double t = (p1p3.Crossed(d2)).Dot(crossProduct) / crossProduct.SquareMagnitude();
	double u = (p1p3.Crossed(d1)).Dot(crossProduct) / crossProduct.SquareMagnitude();

	// 计算交点坐标
	theIntersection = p1_1.Translated(t * d1);

	//if (theIntersection.IsEqual(p1_1, 1e-6) || theIntersection.IsEqual(p1_2, 1e-6))
	{
		// 根据距离裁剪每条线段的起点或终点，使它们在交点处相接
		if (p1_1.Distance(theIntersection) < p1_2.Distance(theIntersection)) {
			theEdge1 = BRepBuilderAPI_MakeEdge(p1_2, theIntersection);
		}
		else {
			theEdge1 = BRepBuilderAPI_MakeEdge(theIntersection, p1_1);
		}

		if (p2_1.Distance(theIntersection) < p2_2.Distance(theIntersection)) {
			theEdge2 = BRepBuilderAPI_MakeEdge(p2_2, theIntersection);
		}
		else {
			theEdge2 = BRepBuilderAPI_MakeEdge(theIntersection, p2_1);
		}
	}
	return true;
}

//延长到交点，交点在base线段上
bool TrimEdgesAtIntersection(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection) {
	// 获取第一条线段的起点和终点
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(theEdge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(theEdge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// 获取第二条线段的起点和终点
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(theEdge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(theEdge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);
	// 计算两条线段的方向向量
	gp_Vec d1(p1_1, p1_2); // 第一条线段的方向向量
	gp_Vec d2(p2_1, p2_2); // 第二条线段的方向向量

	gp_Vec crossProduct = d1.Crossed(d2);

	// 判断是否平行
	if (crossProduct.SquareMagnitude() < 1e-12) {
		return false; // 平行，无交点
	}
	gp_Vec p1p3(p1_1, p2_1);
	// 计算 t 和 u
	double t = (p1p3.Crossed(d2)).Dot(crossProduct) / crossProduct.SquareMagnitude();
	double u = (p1p3.Crossed(d1)).Dot(crossProduct) / crossProduct.SquareMagnitude();

	// 计算交点坐标
	theIntersection = p1_1.Translated(t * d1);
	
	if (theIntersection.IsEqual(p1_1, 1e-6) || theIntersection.IsEqual(p1_2, 1e-6))
	{
		// 根据距离裁剪每条线段的起点或终点，使它们在交点处相接
		if (p1_1.Distance(theIntersection) < p1_2.Distance(theIntersection)) {
			theEdge1 = BRepBuilderAPI_MakeEdge(p1_2, theIntersection);
		}
		else {
			theEdge1 = BRepBuilderAPI_MakeEdge(theIntersection, p1_1);
		}

		if (p2_1.Distance(theIntersection) < p2_2.Distance(theIntersection)) {
			theEdge2 = BRepBuilderAPI_MakeEdge(p2_2, theIntersection);
		}
		else {
			theEdge2 = BRepBuilderAPI_MakeEdge(theIntersection, p2_1);
		}
	}
	
	

	// 根据距离裁剪每条线段的起点或终点，使它们在交点处相接
	/*if (p1_1.Distance(intersection) < p1_2.Distance(intersection)) {
		edge1 = BRepBuilderAPI_MakeEdge(p1_2, intersection);
	}
	else {
		edge1 = BRepBuilderAPI_MakeEdge(intersection, p1_1);
	}

	if (p2_1.Distance(intersection) < p2_2.Distance(intersection)) {
		edge2 = BRepBuilderAPI_MakeEdge(p2_2, intersection);
	}
	else {
		edge2 = BRepBuilderAPI_MakeEdge(intersection, p2_1);
	}*/
	return true;
}
bool judgeless(double x, double y)
{
	double teee = y - x;
	if (y - x > 1e-6)
		return true;
	else
		return false;
}

//判断两条线段相交
bool EdgeIntersect(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2)
{
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge1);  // 获取起点
	TopoDS_Vertex v2 = TopExp::LastVertex(edge1);
	gp_Pnt p1 = BRep_Tool::Pnt(v1);
	gp_Pnt p2 = BRep_Tool::Pnt(v2);

	TopoDS_Vertex w1 = TopExp::FirstVertex(edge2);  // 获取起点
	TopoDS_Vertex w2 = TopExp::LastVertex(edge2);
	gp_Pnt p3 = BRep_Tool::Pnt(w1);
	gp_Pnt p4 = BRep_Tool::Pnt(w2);
	gp_Vec d1(p1, p2); // 第一条线段的方向向量
	gp_Vec d2(p3, p4); // 第二条线段的方向向量
	/*if (p1.X() > p2.X() ? p1.X() : p2.X() < p3.X() < p4.X() ? p3.X() : p4.X() ||
		p1.Y() > p2.Y() ? p1.Y() : p2.Y() < p3.Y() < p4.Y() ? p3.Y() : p4.Y() ||
		p3.X() > p4.X() ? p3.X() : p4.X() < p1.X() < p2.X() ? p1.X() : p2.X() ||
		p3.Y() > p4.Y() ? p3.Y() : p4.Y() < p1.Y() < p2.Y() ? p1.Y() : p2.Y())
	{
		return false;
	}*/
	if (judgeless(p1.X() > p2.X() ? p1.X() : p2.X() , p3.X() < p4.X() ? p3.X() : p4.X()) ||
		judgeless(p1.Y() > p2.Y() ? p1.Y() : p2.Y() , p3.Y() < p4.Y() ? p3.Y() : p4.Y()) ||
			judgeless(p3.X() > p4.X() ? p3.X() : p4.X() , p1.X() < p2.X() ? p1.X() : p2.X()) ||
				judgeless(p3.Y() > p4.Y() ? p3.Y() : p4.Y() , p1.Y() < p2.Y() ? p1.Y() : p2.Y()))
	{
		return false;
	}
	int sideStart = GetPointSide(p1, p2, p3);
	int sideEnd = GetPointSide(p1, p2, p4);

	int endStart = GetPointSide(p3, p4, p1);
	int endEnd = GetPointSide(p3, p4, p2);
	if (sideStart * sideEnd > 0 || endStart * endEnd > 0)
	{
		return false;
	}

	return true;


}

//找到一个面里的交线
bool SheetFalttenMove::relationEdge(TopoDS_Edge edge)
{
	vector<TopoDS_Edge> intersectionLines;
	vector<TopoDS_Edge> lines;
	auto faceIt = m_MapEdgeAdjFaces.find(edge);
	for (auto it : faceIt->second)
	{


		for (TopExp_Explorer edgeExplorer(it, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
		{

			TopoDS_Edge otherEdge = TopoDS::Edge(edgeExplorer.Current());//三维的线
			if (otherEdge != edge)
			{
				gp_Pnt intersection1;
				TopoDS_Edge twoEdge = m_ThreeToTwoEdge.find(otherEdge)->second;//旧二维
				if (EdgeIntersect(twoEdge, m_ThreeToTwoEdge.find(edge)->second))/*AreEdgesIntersecting(twoEdge, ThreeToTwoEdge.find(edge)->second)*/
				{
					intersectionLines.emplace_back(twoEdge);
				}
				else {
					lines.emplace_back(twoEdge);
				}
			}
		}

	}
	interseLine_Line item;
	item.interseLine = intersectionLines;
	item.Line = lines;
	m_interseLine_LineMap[edge] = item;
	return true;
}
void SheetFalttenMove::allReation2dEdge()
{
	vector<TopoDS_Edge> interseEdges;
	for (auto it : m_ThreeToTwoEdge)
	{
		TopoDS_Edge baseEdge = it.second;
		for (auto elem : m_ThreeToTwoEdge)
		{
			TopoDS_Edge comEdge = elem.second;
			if (baseEdge != comEdge)
			{
				if (EdgeIntersect(baseEdge, comEdge) && !isOverlap(baseEdge,comEdge))
				{
					interseEdges.push_back(comEdge);
				}
			}
		}
		m_MapInterseLines[baseEdge] = interseEdges;
		interseEdges.clear();
	}
}
void SheetFalttenMove::allReation2dNewEdge()
{
	vector<TopoDS_Edge> interseEdges;
	for (auto it : m_ThreeToTwoEdge)
	{
		TopoDS_Edge baseEdge = it.second;
		findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, it.second, baseEdge);
		
		for (auto elem : m_ThreeToTwoEdge)
		{
			TopoDS_Edge comEdge = elem.second;
			findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, elem.second, comEdge);
			if (baseEdge != comEdge)
			{
				if (EdgeIntersect(baseEdge, comEdge) && !isOverlap(baseEdge, comEdge))
				{
					/*TopoDS_Edge old2dEdge;
					if(_2dTo3dEdge(m_newMapOldEdge, comEdge, old2dEdge))
					{
						interseEdges.push_back(old2dEdge);
					}
					else
					{
						interseEdges.push_back(comEdge);
					}*/
					interseEdges.push_back(elem.second);
				}
			}
		}
		m_MapInterseNewLines[it.second] = interseEdges;
		interseEdges.clear();
	}
}
void SheetFalttenMove::generateSoltEdge(const TopoDS_Edge& theEdge)
{
	gp_Pnt intersection;
	TopoDS_Edge newbaseEdge = theEdge, newInterseEdge;
	if (m_oldMapNewEdge.find(theEdge) != m_oldMapNewEdge.end())
	{
		newbaseEdge = m_oldMapNewEdge.find(theEdge)->second[0];
	}
	if (m_MapInterseNewLines.find(theEdge) != m_MapInterseNewLines.end())
	{
		for (auto itemm : m_MapInterseNewLines.find(theEdge)->second)
		{
			vector<TopoDS_Edge> aEdge, aVectorEdge1;
			newInterseEdge = itemm;
			if (m_oldMapNewEdge.find(itemm) != m_oldMapNewEdge.end())
			{
				newInterseEdge = m_oldMapNewEdge.find(itemm)->second[0];
				if (m_oldMapNewEdge.find(itemm)->second.size() >= 3)
				{
					newInterseEdge = m_oldMapNewEdge.find(itemm)->second[2];
				}
			}

			TrimEdgesAtIntersection1(newbaseEdge, newInterseEdge, intersection);//延长线到交点
			//oldMapNewEdge[edge] = newsideEdge;
			aEdge.emplace_back(newInterseEdge);
			if (m_oldMapNewEdge.find(itemm) != m_oldMapNewEdge.end())
			{
				if (m_oldMapNewEdge.find(itemm)->second.size() >= 3)
				{
					aEdge.clear();
					aEdge.emplace_back(m_oldMapNewEdge.find(itemm)->second[0]);
					aEdge.emplace_back(m_oldMapNewEdge.find(itemm)->second[1]);
					aEdge.emplace_back(newInterseEdge);
				}
			}
			m_oldMapNewEdge[itemm] = aEdge;
			aEdge.clear();
		}
	}
}

void SheetFalttenMove::generateOverlapEdge(const TopoDS_Edge& theEdge)
{
	gp_Pnt intersection;
	TopoDS_Edge newbaseEdge = theEdge, newInterseEdge;
	if (m_oldMapNewEdge.find(theEdge) != m_oldMapNewEdge.end())
	{
		newbaseEdge = m_oldMapNewEdge.find(theEdge)->second[0];
		if (m_oldMapNewEdge.find(theEdge)->second.size() >= 3)
		{
			newbaseEdge = m_oldMapNewEdge.find(theEdge)->second[2];
		}
	}
	if (m_MapInterseNewLines.find(theEdge) != m_MapInterseNewLines.end())
	{
		for (auto itemm : m_MapInterseNewLines.find(theEdge)->second)
		{
			vector<TopoDS_Edge> aEdge, aVectorEdge1;
			if (m_overlapEdges.find(itemm) == m_overlapEdges.end())
			{
				continue;
			}
			newInterseEdge = itemm;
			if (m_oldMapNewEdge.find(itemm) != m_oldMapNewEdge.end())
			{
				newInterseEdge = m_oldMapNewEdge.find(itemm)->second[0];
				if (m_oldMapNewEdge.find(itemm)->second.size() >= 3)
				{
					newInterseEdge = m_oldMapNewEdge.find(itemm)->second[2];
				}
			}

			TrimEdgesAtIntersection1(newbaseEdge, newInterseEdge, intersection);//延长线到交点
			//oldMapNewEdge[edge] = newsideEdge;
			aEdge.emplace_back(newInterseEdge);
			if (m_oldMapNewEdge.find(itemm) != m_oldMapNewEdge.end())
			{
				if (m_oldMapNewEdge.find(itemm)->second.size() >= 3)
				{
					aEdge.clear();
					aEdge.emplace_back(m_oldMapNewEdge.find(itemm)->second[0]);
					aEdge.emplace_back(m_oldMapNewEdge.find(itemm)->second[1]);
					aEdge.emplace_back(newInterseEdge);
				}
			}
			m_oldMapNewEdge[itemm] = aEdge;
			m_overLapEdge_Point[itemm] = intersection;
			aEdge.clear();
		}
	}
}
void SheetFalttenMove::generateEdge(const TopoDS_Edge &theEdge)
{
	gp_Pnt intersection;
	TopoDS_Edge newbaseEdge = theEdge, newInterseEdge;
	if (m_oldMapNewEdge.find(theEdge) != m_oldMapNewEdge.end())
	{
		newbaseEdge = m_oldMapNewEdge.find(theEdge)->second[0];
	}
	if (m_MapInterseNewLines.find(theEdge) != m_MapInterseNewLines.end())
	{
		for (auto itemm : m_MapInterseNewLines.find(theEdge)->second)
		{
			vector<TopoDS_Edge> aEdge,aVectorEdge1;
			TopoDS_Edge aEdge1, aEdge2;
			newInterseEdge = itemm;
			if (m_oldMapNewEdge.find(itemm) != m_oldMapNewEdge.end())
			{
				newInterseEdge = m_oldMapNewEdge.find(itemm)->second[0];
				if (m_oldMapNewEdge.find(itemm)->second.size() >= 3)
				{
					aEdge1 = m_oldMapNewEdge.find(itemm)->second[1];
					aEdge2 = m_oldMapNewEdge.find(itemm)->second[2];
				}
			}
			
			TrimEdgesAtIntersection(newbaseEdge, newInterseEdge, intersection);//延长线到交点
			//oldMapNewEdge[edge] = newsideEdge;
			aEdge.emplace_back(newInterseEdge);
			if (m_oldMapNewEdge.find(itemm) != m_oldMapNewEdge.end())
			{
				if (m_oldMapNewEdge.find(itemm)->second.size() >= 3)
				{
					aEdge.emplace_back(m_oldMapNewEdge.find(itemm)->second[1]);
					aEdge.emplace_back(m_oldMapNewEdge.find(itemm)->second[2]);
				}
			}
			m_oldMapNewEdge[itemm] = aEdge;
			aEdge.clear();
		}
	}
}
// 函数计算两面夹角
bool SheetFalttenMove::CalculateAngleBetweenFaces(const TopoDS_Face& theBaseFace,const TopoDS_Face &theTarFace, double& angle) {

	Handle(Geom_Surface) surface1 = BRep_Tool::Surface(theBaseFace);
	Handle(Geom_Surface) surface2 = BRep_Tool::Surface(theTarFace);
	// 假设 surface1 和 surface2 是 Geom_Surface 类型的两个平面
	GeomLProp_SLProps props1(surface1, 0.5, 0.5, 1, Precision::Confusion());
	GeomLProp_SLProps props2(surface2, 0.5, 0.5, 1, Precision::Confusion());

	// 获取两个平面的法线向量
	gp_Dir normal1 = props1.Normal();
	gp_Dir normal2 = props2.Normal();
	if (normal1.IsEqual(normal2, 1e-6))
	{
		angle = M_PI;
		return true;
	}
	gp_Dir crossNormal = normal1.Crossed(normal2);
	if (gp_Vec(crossNormal).Magnitude() < gp::Resolution())
	{
		return false;
	}
	// 计算夹角（弧度）
	angle = normal1.Angle(normal2);

	// 判断两个Face是正折还是反折

	// 计算形心，创建投影平面plane
	GProp_GProps gpropsBase;
	BRepGProp::SurfaceProperties(theBaseFace, gpropsBase);
	gp_Pnt centerBase = gpropsBase.CentreOfMass();

	GProp_GProps gpropsTarget;
	BRepGProp::SurfaceProperties(theTarFace, gpropsTarget);
	gp_Pnt centerTarget = gpropsTarget.CentreOfMass();

	gp_Pln plane(centerBase, crossNormal);

	//按照法线方向将形心平移，定义一个新的点
	// 设定平移距离
	Standard_Real distance = 100.0;

	gp_Vec transVecBase = distance * gp_Vec(normal1);
	gp_Pnt centerBaseTrs = centerBase.Translated(transVecBase);

	gp_Vec transVecTarget = distance * gp_Vec(normal2);
	gp_Pnt centerTargetTrs = centerTarget.Translated(transVecTarget);

	//将四个点投影到plane上
	gp_Pnt2d centerBasePrj = ProjLib::Project(plane, centerBase);
	gp_Pnt2d centerBaseTrsPrj = ProjLib::Project(plane, centerBaseTrs);
	gp_Pnt2d centerTargetPrj = ProjLib::Project(plane, centerTarget);
	gp_Pnt2d centerTargetTrsPrj = ProjLib::Project(plane, centerTargetTrs);

	gp_Lin2d lineBasePrj(centerBasePrj, gp_Vec2d(centerBasePrj, centerBaseTrsPrj));
	gp_Lin2d lineTargetPrj(centerTargetPrj, gp_Vec2d(centerTargetPrj, centerTargetTrsPrj));

	IntAna2d_AnaIntersection Int2d(lineBasePrj, lineTargetPrj);
	if (!Int2d.IsDone())
	{
		return false;
	}

	int nbpts = Int2d.NbPoints();

	gp_Pnt2d intersectionPnt(Int2d.Point(1).Value());

	gp_Vec2d vecBase2d(centerBasePrj, centerBaseTrsPrj);
	gp_Vec2d vecInt2d(centerBasePrj, intersectionPnt);

	double angle2d = vecBase2d.Angle(vecInt2d);

	if (fabs(angle2d) > 1e-4)
	{
		angle = angle + M_PI;
		//angle = angle * 180.0 / M_PI + 180;
	}

	//cout << angle * 180.0 / M_PI << endl;

	return true;
}
// 函数计算两面夹角
bool SheetFalttenMove::CalculateAngleBetweenFaces(const vector<TopoDS_Face>& face, double& angle) {

	Handle(Geom_Surface) surface1 = BRep_Tool::Surface(face[0]);
	Handle(Geom_Surface) surface2 = BRep_Tool::Surface(face[1]);
	// 假设 surface1 和 surface2 是 Geom_Surface 类型的两个平面
	GeomLProp_SLProps props1(surface1, 0.5, 0.5, 1, Precision::Confusion());
	GeomLProp_SLProps props2(surface2, 0.5, 0.5, 1, Precision::Confusion());

	// 获取两个平面的法线向量
	gp_Dir normal1 = props1.Normal();
	gp_Dir normal2 = props2.Normal();
	gp_Dir crossNormal = normal1.Crossed(normal2);
	if (gp_Vec(crossNormal).Magnitude() < gp::Resolution())
	{
		return false;
	}
	// 计算夹角（弧度）
	angle = normal1.Angle(normal2);

	// 判断两个Face是正折还是反折

	// 计算形心，创建投影平面plane
	GProp_GProps gpropsBase;
	BRepGProp::SurfaceProperties(face[0], gpropsBase);
	gp_Pnt centerBase = gpropsBase.CentreOfMass();

	GProp_GProps gpropsTarget;
	BRepGProp::SurfaceProperties(face[1], gpropsTarget);
	gp_Pnt centerTarget = gpropsTarget.CentreOfMass();

	gp_Pln plane(centerBase, crossNormal);

	//按照法线方向将形心平移，定义一个新的点
	// 设定平移距离
	Standard_Real distance = 100.0;

	gp_Vec transVecBase = distance * gp_Vec(normal1);
	gp_Pnt centerBaseTrs = centerBase.Translated(transVecBase);

	gp_Vec transVecTarget = distance * gp_Vec(normal2);
	gp_Pnt centerTargetTrs = centerTarget.Translated(transVecTarget);

	//将四个点投影到plane上
	gp_Pnt2d centerBasePrj = ProjLib::Project(plane, centerBase);
	gp_Pnt2d centerBaseTrsPrj = ProjLib::Project(plane, centerBaseTrs);
	gp_Pnt2d centerTargetPrj = ProjLib::Project(plane, centerTarget);
	gp_Pnt2d centerTargetTrsPrj = ProjLib::Project(plane, centerTargetTrs);

	gp_Lin2d lineBasePrj(centerBasePrj, gp_Vec2d(centerBasePrj, centerBaseTrsPrj));
	gp_Lin2d lineTargetPrj(centerTargetPrj, gp_Vec2d(centerTargetPrj, centerTargetTrsPrj));

	IntAna2d_AnaIntersection Int2d(lineBasePrj, lineTargetPrj);
	if (!Int2d.IsDone())
	{
		return false;
	}

	int nbpts = Int2d.NbPoints();

	gp_Pnt2d intersectionPnt(Int2d.Point(1).Value());

	gp_Vec2d vecBase2d(centerBasePrj, centerBaseTrsPrj);
	gp_Vec2d vecInt2d(centerBasePrj, intersectionPnt);

	double angle2d = vecBase2d.Angle(vecInt2d);

	if (fabs(angle2d) > 1e-4)
	{
		angle = angle + M_PI;
		//angle = angle * 180.0 / M_PI + 180;
	}

	//cout << angle * 180.0 / M_PI << endl;

	return true;
}
void SheetFalttenMove::wrapAngle()
{
	map<TopoDS_Edge, TopoDS_Edge> aResult;//三维的边
	findSameEdge(aResult);
	vector<TopoDS_Face> aBaseFaceResult, aTarFaceResult;
	double distance = 2., angle;
	for (auto elem : aResult)
	{
		if (getAdjFace(elem.first, aBaseFaceResult) && getAdjFace(elem.second, aTarFaceResult))
		{
			if (aBaseFaceResult.size() <= 2 && aTarFaceResult.size() <= 2)
			{
				CalculateAngleBetweenFaces(aBaseFaceResult[0], aTarFaceResult[0], angle);

				calRetractTranslate(elem.first, aBaseFaceResult[0], distance, angle);
			}
		}
	}
}
void SheetFalttenMove::generate(TopoDS_Compound& aCompound)
{
	allReation2dEdge();
	processOverlap();
	allReation2dNewEdge();
	//for (auto elem : m_MapEdgeAdjFaces)
	//{
	//	if (m_finishEdge.find(elem.first) == m_finishEdge.end())
	//	{
	//		moveEdge( elem.first, aCompound);
	//		//core->aFalttenMove.generateEdge();
	//	}
	//}

	for (auto elem : m_ThreeToTwoEdge)
	{
		if (m_finishEdge.find(elem.first) == m_finishEdge.end())
		{
			moveEdge(elem.first, aCompound);
			//core->aFalttenMove.generateEdge();
		}
	}

	makeLapelMap();//折边
	translateLapel(1.);
	wrapAngle();
	set<TopoDS_Edge> FinishEdges;
	for (auto elem : m_overlapEdges)
	{
		if (FinishEdges.find(elem) != FinishEdges.end())
		{
			continue;
		}
		TopoDS_Edge aBaseEdge = elem;
		for (auto it : m_overlapEdges)
		{
			TopoDS_Edge aTarEdge = it;
			if (aBaseEdge != aTarEdge)
			{
				if (EdgeIntersect(aBaseEdge, aTarEdge))
				{
					if (m_overLapEdge_Point.find(aBaseEdge) != m_overLapEdge_Point.end() && m_overLapEdge_Point.find(aTarEdge) != m_overLapEdge_Point.end())
					{
						vector<TopoDS_Edge> aVectorEdges;
						gp_Pnt aBasePoint = m_overLapEdge_Point.find(aBaseEdge)->second;
						gp_Pnt aTarPoint = m_overLapEdge_Point.find(aTarEdge)->second;
						TopoDS_Edge aNewEdge = BRepBuilderAPI_MakeEdge(aBasePoint, aTarPoint);
						if (findMap_EdgeAndVector_ToVector(m_oldMapNewEdge, aBaseEdge, aVectorEdges))
						{
							FinishEdges.insert(it);
							aVectorEdges.emplace_back(aNewEdge);
							m_oldMapNewEdge[aBaseEdge] = aVectorEdges;
						}
						
					}
				}

			}
		}
	}
	/*makeLapelMap();
	translateLapel(1.);
	wrapAngle();*/

}
void SheetFalttenMove::generateNewEdge(const vector<TopoDS_Edge>& theMoveEdge, const gp_Trsf& translation , bool theMove)
{
	for (auto it : theMoveEdge)
	{
		vector<TopoDS_Edge> aEdges;
		vector<TopoDS_Edge> tempedge;
		if (m_oldMapNewEdge.find(it) != m_oldMapNewEdge.end())
		{
			tempedge = m_oldMapNewEdge.find(it)->second;
		}
		else {
			tempedge.emplace_back(it);
		}
		if(theMove)
		{
			if (m_overLapEdge_Point.find(it) != m_overLapEdge_Point.end())
			{
				gp_Pnt aBasePoint = m_overLapEdge_Point.find(it)->second;
				aBasePoint.Transform(translation);
				m_overLapEdge_Point[it] = aBasePoint;
			}
			for (auto elem : tempedge)
			{
				TopoDS_Edge newedge = TranslateEdge(elem, translation);
				aEdges.emplace_back(newedge);
			}
			m_oldMapNewEdge[it] = aEdges;
			aEdges.clear();
		}
		else
		{
			m_oldMapNewEdge[it] = tempedge;
		}

	}
}
void SheetFalttenMove::generateMidleEdge(const vector<TopoDS_Edge> &theMoveEdge, const gp_Trsf &theTranslation ,const gp_Trsf & theTranslation_slot)
{
	for (auto it : theMoveEdge)//遍历共线的线
	{
		vector<TopoDS_Edge> aEdges;
		if (m_TwoToThreeEdge.find(it) != m_TwoToThreeEdge.end() && m_MapEdgeAdjFaces.find(m_TwoToThreeEdge.find(it)->second)->second.size()>=2)
		{
			m_finishEdge.insert(m_TwoToThreeEdge.find(it)->second);
			TopoDS_Edge tempedge;
			if (m_oldMapNewEdge.find(it) != m_oldMapNewEdge.end())
			{
				tempedge = m_oldMapNewEdge.find(it)->second[0];
			}
			else {
				tempedge = it;
			}
			TopoDS_Edge newedge;
			if (m_slotEdges_2d.find(it) != m_slotEdges_2d.end())
			{
				newedge = TranslateEdge(tempedge, theTranslation_slot);
			}
			else
			{
				newedge = TranslateEdge(tempedge, theTranslation);
			}
			TopoDS_Edge midedge = midleLine(tempedge, newedge);
			aEdges.emplace_back(midedge);
			aEdges.emplace_back(newedge);
			aEdges.emplace_back(tempedge);
			m_oldMapNewEdge[it] = aEdges;
		}
		else
		{
			TopoDS_Edge tempedge;
			if (m_oldMapNewEdge.find(it) != m_oldMapNewEdge.end())
			{
				tempedge = m_oldMapNewEdge.find(it)->second[0];
			}
			else {
				tempedge = it;
			}
			TopoDS_Edge newedge = TranslateEdge(tempedge, theTranslation);
			TopoDS_Edge midedge = midleLine(tempedge, newedge);
			//aEdges.emplace_back(midedge);
			//aEdges.emplace_back(newedge);
			aEdges.emplace_back(tempedge);
			m_oldMapNewEdge[it] = aEdges;
		}
		/*TopoDS_Edge tempedge;
		if (m_oldMapNewEdge.find(it) != m_oldMapNewEdge.end())
		{
			tempedge = m_oldMapNewEdge.find(it)->second[0];
		}
		else {
			tempedge = it;
		}
		TopoDS_Edge newedge = TranslateEdge(tempedge, translation);
		TopoDS_Edge midedge = midleLine(tempedge, newedge);
		aEdges.emplace_back(midedge);
		aEdges.emplace_back(newedge);
		aEdges.emplace_back(tempedge);
		m_oldMapNewEdge[it] = aEdges;*/
	}
}
void SheetFalttenMove::moveEdge(TopoDS_Edge edge,TopoDS_Compound& aCompound)
{
	BRep_Builder aBuilder;
	auto faceIt = m_MapEdgeAdjFaces.find(edge);
	vector<TopoDS_Edge> leftEdges;
	vector<TopoDS_Edge> rightEdges;
	vector<TopoDS_Edge> colines;
	vector<TopoDS_Edge> otherlines;
	vector<TopoDS_Face> faces;
	for (auto it : faceIt->second)
	{
		faces.emplace_back(it);
	}
	if (faceIt != m_MapEdgeAdjFaces.end() && faceIt->second.size() >= 2)
	{

		m_finishEdge.insert(edge);
		gp_Trsf translation,translation_slot;
		Standard_Real angle;
		CalculateAngleBetweenFaces(faces,angle);
	
		TopoDS_Edge baseEdge = m_ThreeToTwoEdge.find(edge)->second;//旧二维
		FindEdgesOnBothSides(m_ThreeToTwoEdge, baseEdge, leftEdges, rightEdges, colines, otherlines);//所有容器装的都是旧二维,朝里挪
		/*if (angle > M_PI)
		{
			m_baseVec = -m_baseVec;
		}*/
		double moveDis = 2.;
		calTranslate(moveDis, translation_slot, angle);
		calTranslate(moveDis, translation ,angle);//确定移动方向和角度
		if (angle > M_PI)
		{
			m_isPositiveFold = false;
		}
		else
		{
			m_isPositiveFold = true;
		}
		if (rightEdges.size() <= leftEdges.size())
		{
			generateMidleEdge(colines, translation, translation_slot);
			generateNewEdge(rightEdges, translation, true);
			generateNewEdge(otherlines, translation, true);
			generateNewEdge(leftEdges, translation, false);

		}
		else
		{
			translation = translation.Inverted();
			translation_slot = translation_slot.Inverted();
			generateMidleEdge(colines, translation, translation_slot);
			generateNewEdge(rightEdges, translation, false);
			generateNewEdge(otherlines, translation, true);
			generateNewEdge(leftEdges, translation, true);
		}
		/*generateMidleEdge(colines, translation);
		generateNewEdge(rightEdges, translation ,true);
		generateNewEdge(otherlines, translation,false);
		generateNewEdge(leftEdges, translation, false);*/


		for (auto it : m_ThreeToTwoEdge)
		{
			TopoDS_Edge edge;
			vector<TopoDS_Edge> aEdges;
			edge = it.second;
			//aBuilder.Add(aCompound1, edge);
			if (m_oldMapNewEdge.find(edge) != m_oldMapNewEdge.end())
			{
				aEdges = m_oldMapNewEdge.find(edge)->second;
				for (auto elem : aEdges)
				{
					aBuilder.Add(aCompound, elem);
				}
			}
			else
			{
				aBuilder.Add(aCompound, edge);
			}
		}
		for (auto it : colines)//遍历共线的线
		{
			if (m_isPositiveFold)
			{
				//generateEdge(it);
				generateSoltEdge(it);
			}
			generateOverlapEdge(it);
			//generateEdge(it);
		}

		//FlattenFace::BuildDocStd(aCompound);

		FlattenFace::BuildDocStd(aCompound);
		aCompound.Nullify(); // 将 compound 置为无效状态
		aBuilder.MakeCompound(aCompound);

	}
}
bool SheetFalttenMove::getAdjFace(TopoDS_Edge edge, vector<TopoDS_Face>& result)
{
	auto it = m_MapEdgeAdjFaces.find(edge);
	if (it != m_MapEdgeAdjFaces.end())
	{
		vector<TopoDS_Face> temp(it->second.begin(), it->second.end());
		result = temp;

		return true;
	}
	return false;
}
//端点是否相同
bool SheetFalttenMove::AreEdgesSameEndpoints(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2) {
	// 获取第一条边的两个端点
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// 获取第二条边的两个端点
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);

	// 判断两条边的端点是否相同（无论顺序）
	bool sameStartEnd = p1_1.IsEqual(p2_1, Precision::Confusion()) && p1_2.IsEqual(p2_2, Precision::Confusion());
	bool sameEndStart = p1_1.IsEqual(p2_2, Precision::Confusion()) && p1_2.IsEqual(p2_1, Precision::Confusion());

	return sameStartEnd || sameEndStart;
}
void SheetFalttenMove::findSameEdge(map<TopoDS_Edge,TopoDS_Edge> &result)
{
	TopoDS_Edge aBaseEdge, aTarEdge;
	set<TopoDS_Edge> aMapFinishEdge;
	for (auto elem : m_ThreeToTwoEdge)
	{
		aBaseEdge = elem.first;
		//aMapFinishEdge.insert(aBaseEdge);
		for (auto it : m_ThreeToTwoEdge)
		{
			aTarEdge = it.first;
			if (aMapFinishEdge.find(aTarEdge) != aMapFinishEdge.end() || aBaseEdge == aTarEdge)
				continue;
			if (AreEdgesSameEndpoints(aBaseEdge,aTarEdge))
			{
				result[aBaseEdge] = aTarEdge;
				aMapFinishEdge.insert(aBaseEdge);
				aMapFinishEdge.insert(aTarEdge);
			}
			//aMapFinishEdge.insert(aTarEdge);
		}
	}
	return;
}
#endif