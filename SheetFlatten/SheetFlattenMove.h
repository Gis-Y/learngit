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
	map<TopoDS_Edge, vector<TopoDS_Edge>>m_MapInterseLines;//�ɶ�ά�Ծɶ�ά
	map<TopoDS_Edge, vector<TopoDS_Edge>>m_MapInterseNewLines;//�ɶ�ά�Ծɶ�ά��ֻ�����жϽ��ߵ�ʱ���õ����¶�ά
	map<TopoDS_Edge, interseLine_Line> m_interseLine_LineMap;//��ά�Ծɶ�ά
	map<TopoDS_Edge, vector<TopoDS_Edge>> m_MapLapelEdges;
	set<TopoDS_Edge> m_finishEdge;
	set<TopoDS_Edge> m_overlapEdges;
	map<TopoDS_Edge, gp_Pnt> m_overLapEdge_Point;
	set<TopoDS_Face> m_twoFaces;
	vector<TopoDS_Edge> m_positiveEdges;
	vector<TopoDS_Edge> m_negativeEdges;
	vector<TopoDS_Edge> m_slotEdges_3d;//��ά��
	set<TopoDS_Edge> m_slotEdges_2d;
	gp_Dir m_refDir;
	gp_Vec m_baseVec;//�ƶ�����  ��
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
		// ʹ��XCAFDoc_ColorType::XCAFDoc_ColorCurves�趨�ߵ���ɫ
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


	theTranslation.SetTranslation(translationVec);  // ����ƽ�ƾ���

	return;
}
//�ҵ��߶ε���һ��
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
		aLapelEdge1 = item.second[0];//�϶�ά
		aLapelEdge2 = item.second[1];

		if (m_overlapEdges.find(aLapelEdge1) != m_overlapEdges.end() || m_overlapEdges.find(aLapelEdge2) != m_overlapEdges.end())
		{
			continue;
		}
		IsEdgesAtIntersection(aLapelEdge1, aLapelEdge2, aOldInterPoint);

		aOldPoint1 = findOtherPoint(aLapelEdge1, aOldInterPoint);
		aOldPoint2 = findOtherPoint(aLapelEdge2, aOldInterPoint);
		aOldLapelInterEdge1 = findAdjEdge(aLapelEdge1, aOldPoint1);//�϶�ά
		aOldLapelInterEdge2 = findAdjEdge(aLapelEdge2, aOldPoint2);
		findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, aOldLapelInterEdge1, aNewLapelInterEdge1);//�¶�ά
		findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, aOldLapelInterEdge2, aNewLapelInterEdge2);

		findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, aLapelEdge1, aLapelNewEdge1);//�¶�ά
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
	// ���� surface1 �� surface2 �� Geom_Surface ���͵�����ƽ��
	GeomLProp_SLProps props1(surface1, 0.5, 0.5, 1, Precision::Confusion());
	GeomLProp_SLProps props2(surface2, 0.5, 0.5, 1, Precision::Confusion());

	// ��ȡ����ƽ��ķ�������
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

//�󽻵�����
bool SheetFalttenMove::IsEdgesAtIntersection(TopoDS_Edge& edge1, TopoDS_Edge& edge2, gp_Pnt& intersection) {
	// ��ȡ��һ���߶ε������յ�
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// ��ȡ�ڶ����߶ε������յ�
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);
	// ���������߶εķ�������
	gp_Vec d1(p1_1, p1_2); // ��һ���߶εķ�������
	gp_Vec d2(p2_1, p2_2); // �ڶ����߶εķ�������

	gp_Vec crossProduct = d1.Crossed(d2);

	// �ж��Ƿ�ƽ��
	if (crossProduct.SquareMagnitude() < 1e-12) {
		return false; // ƽ�У��޽���
	}
	gp_Vec p1p3(p1_1, p2_1);
	// ���� t �� u
	double t = (p1p3.Crossed(d2)).Dot(crossProduct) / crossProduct.SquareMagnitude();
	double u = (p1p3.Crossed(d1)).Dot(crossProduct) / crossProduct.SquareMagnitude();

	// ���㽻������
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
	// ��ȡ�߶�1�Ķ˵�
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// ��ȡ�߶�2�Ķ˵�
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);

	// ���㷽������
	gp_Vec d1(p1_1, p1_2);
	gp_Vec d2(p2_1, p2_2);

	// �жϷ��������Ƿ��� (����Ƿ�ӽ���)
	gp_Vec crossProduct = d1.Crossed(d2);
	if (crossProduct.SquareMagnitude() > 1e-6) {
		// ��������Ϊ�㣬˵��������
		return false;
	}

	// ����Ƿ���ͬһֱ���ϣ����ԱȽ�������ķ�������
	gp_Vec dir1(p1_1, p2_1);
	gp_Vec dir2(p1_1, p2_2);
	if (std::abs(dir1.Crossed(d1).SquareMagnitude()) > 1e-6) {
		return false; // ����ͬһֱ��
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
	// ���ĸ��˵�洢��������
	std::vector<gp_Pnt> points = { p1_1, p1_2, p2_1, p2_2 };
	gp_Pnt aPoint1;
	bool aIsInter = false;
	// �������߶η�����ͶӰ�ľ�������
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
	// ��ȡ�߶�1�Ķ˵�
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// ��ȡ�߶�2�Ķ˵�
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);

	// ���㷽������
	gp_Vec d1(p1_1, p1_2);
	gp_Vec d2(p2_1, p2_2);

	// �жϷ��������Ƿ��� (����Ƿ�ӽ���)
	gp_Vec crossProduct = d1.Crossed(d2);
	if (crossProduct.SquareMagnitude() > 1e-6) {
		// ��������Ϊ�㣬˵��������
		return false;
	}

	// ����Ƿ���ͬһֱ���ϣ����ԱȽ�������ķ�������
	gp_Vec dir1(p1_1, p2_1);
	gp_Vec dir2(p1_1, p2_2);
	if (std::abs(dir1.Crossed(d1).SquareMagnitude()) > 1e-3) {
		return false; // ����ͬһֱ��
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
	// ���ĸ��˵�洢��������
	std::vector<gp_Pnt> points = { p1_1, p1_2, p2_1, p2_2 };
	gp_Pnt aPoint1;
	bool aIsInter = false;
	// �������߶η�����ͶӰ�ľ�������
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
			// ������ p1 �� p2 ������
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
	// ��ȡ�ߵ������˵�
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge);
	TopoDS_Vertex v2 = TopExp::LastVertex(edge);

	// ��ȡ�˵�����
	gp_Pnt p1 = BRep_Tool::Pnt(v1);
	gp_Pnt p2 = BRep_Tool::Pnt(v2);

	// ����ߵķ�������
	gp_Vec d(p1, p2);

	// ����� p1 �� point ������
	gp_Vec pointVector(p1, point);

	// ����㵽�ߵ�ͶӰϵ��
	double t = pointVector.Dot(d) / d.SquareMagnitude();

	// ���� t �ķ�Χ�� [0, 1] �ڣ�ȷ��ͶӰ���ڱ���
	if (t < 0) {
		return p1; // ���ض˵� p1
	}
	else if (t > 1) {
		return p2; // ���ض˵� p2
	}

	// ����ͶӰ��
	gp_Pnt projection = p1.Translated(d.Multiplied(t));
	return projection;
}

//ͨ�������ҽ���
TopoDS_Edge SheetFalttenMove::findAdjEdge(const TopoDS_Edge baseEdge, const gp_Pnt basePoint)
{
	if (m_MapInterseLines.find(baseEdge) != m_MapInterseLines.end())
	{
		for (auto it : m_MapInterseLines.find(baseEdge)->second)
		{
			// ��ȡ�ߵ������˵�
			TopoDS_Vertex v1 = TopExp::FirstVertex(it);
			TopoDS_Vertex v2 = TopExp::LastVertex(it);

			// ��ȡ�˵�����
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
// ��������㵽ֱ�ߵĴ��㽻��
gp_Pnt GetPerpendicularFootPoint(const gp_Pnt& point, const TopoDS_Edge& edge)
{
	// ��ȡֱ�ߵ������˵�
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge);
	TopoDS_Vertex v2 = TopExp::LastVertex(edge);
	gp_Pnt p1 = BRep_Tool::Pnt(v1);
	gp_Pnt p2 = BRep_Tool::Pnt(v2);

	// ����ֱ�ߵķ�������
	gp_Vec lineVec(p1, p2);

	// ����� P ��ֱ���ϵ�ͶӰ
	gp_Vec p1ToPoint(p1, point);  // ���� P1 ���������
	double projectionLength = p1ToPoint.Dot(lineVec) / lineVec.SquareMagnitude();

	// ����ͶӰ���ȼ��㴹�������
	gp_Vec projectionVec = lineVec.Scaled(projectionLength);  // ����ͶӰ����
	gp_Pnt intersectionPoint = p1.Translated(projectionVec);  // ͶӰ����Ǵ���

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
					oldTarEdge1_2d = findAdjEdge(it.second, aInterPoint2);//��������
					oldTarEdge2_2d = findAdjEdge(item.second, aInterPoint1);
					m_overlapEdges.insert(oldTarEdge1_2d);
					m_overlapEdges.insert(oldTarEdge2_2d);

					gp_Pnt aPoint1,aPoint2;
					IsEdgesAtIntersection(oldTarEdge1_2d,oldTarEdge2_2d, aPoint1);//�ҵ�����
					aPoint2 = aPoint1;
					// ���ŵ�λ���������ݸ����ľ�������λ������
					
					double distance = 3.;
					gp_Vec displacement1 = aDirvec1.Scaled(distance);	
					// ����ʼ�� p1 ���ŷ��������ƶ��������µĵ�
					aPoint1 = aPoint1.Translated(displacement1);//�ƶ���Ľ���
					gp_Pnt vecticalPoint1 = GetPerpendicularFootPoint(aPoint1, tempEdge1);//���߽���
					newTarEdge1 = BRepBuilderAPI_MakeEdge(aPoint1, vecticalPoint1);
					tempEdge1 = BRepBuilderAPI_MakeEdge(aStartPoint1, vecticalPoint1);


					gp_Vec displacement2 = aDirvec2.Scaled(distance);
					// ����ʼ�� p1 ���ŷ��������ƶ��������µĵ�
					aPoint2 = aPoint2.Translated(displacement2);
					gp_Pnt vecticalPoint2 = GetPerpendicularFootPoint(aPoint2, tempEdge2);
					newTarEdge2 = BRepBuilderAPI_MakeEdge(aPoint2, vecticalPoint2);
					tempEdge2 = BRepBuilderAPI_MakeEdge(vecticalPoint2, aStartPoint2);
					
					//ͨ��2d���ҵ�3d�ߣ��ҵ��棬�ҵ����ߵ���
					
					
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
		// ���� surface1 �� surface2 �� Geom_Surface ���͵�����ƽ��
		GeomLProp_SLProps props1(surface1, 0.5, 0.5, 1, Precision::Confusion());
		GeomLProp_SLProps props2(surface2, 0.5, 0.5, 1, Precision::Confusion());

		// ��ȡ����ƽ��ķ�������
		gp_Dir normal1 = props1.Normal();
		gp_Dir normal2 = props2.Normal();

		// ���� referenceVec ��ƽ�潻�ߵķ�������
		refDir = normal1 ^ normal2;
	}
}
*/
int GetPointSide1(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point) {
	gp_Vec baseVec(baseStart, baseEnd);  // ��׼�ߵķ�������
	gp_Vec vecToPoint(baseStart, point); // �ӻ�׼�����ָ��ǰ�������

	// ��������Z�������жϵ��ڻ�׼�ߵ���һ��
	gp_Vec crossProduct = baseVec ^ vecToPoint;

	if (crossProduct.Z() > 1e-3) {
		return 1;  // ���ڻ�׼�ߵ�һ��
	}
	else if (crossProduct.Z() < -1e-3) {
		return -1; // ���ڻ�׼�ߵ���һ��
	}
	return 0;  // ��������
}
int GetPointSide(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point) {
	gp_Vec baseVec(baseStart, baseEnd);  // ��׼�ߵķ�������
	gp_Vec vecToPoint(baseStart, point); // �ӻ�׼�����ָ��ǰ�������

	// ��������Z�������жϵ��ڻ�׼�ߵ���һ��
	gp_Vec crossProduct = baseVec ^ vecToPoint;

	if (crossProduct.Z() > 5.) {
		return 1;  // ���ڻ�׼�ߵ�һ��
	}
	else if (crossProduct.Z() < -5.) {
		return -1; // ���ڻ�׼�ߵ���һ��
	}
	return 0;  // ��������
}

//�����ƶ�����
gp_Vec SheetFalttenMove::CalMoveVector(const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p) {
	gp_Vec lineVec(p1, p2); // �� p1 �� p2 ������

	gp_Vec perpendicularVec = gp_Vec(0, 0, 1) ^ lineVec; // ���㴹ֱ��lineVec���������������lineVec����Z�᷽��

	gp_Vec pointVec(p2, p); // �� p2 �� p ������

	if (pointVec.Dot(perpendicularVec) < 0) {
		perpendicularVec *= -1; // ȷ����ֱ����ָ��� p
	}
	processFlotInVec(perpendicularVec);
	perpendicularVec.Normalize();

	return perpendicularVec; // ���ص�λ����
}

// �������ҵ�λ�ڻ�׼����������б�
void SheetFalttenMove::FindEdgesOnBothSides(const map< TopoDS_Edge, TopoDS_Edge> mapEdges, const TopoDS_Edge& baseEdge,
	std::vector<TopoDS_Edge>& leftEdges, std::vector<TopoDS_Edge>& rightEdges, std::vector<TopoDS_Edge>& colines, std::vector<TopoDS_Edge>& otherlines) {
	// ��ȡ��׼�ߵ������յ�
	TopoDS_Edge aBaseEdge = baseEdge;//�ɶ�ά
	if (m_oldMapNewEdge.find(aBaseEdge) != m_oldMapNewEdge.end())
	{
		aBaseEdge = m_oldMapNewEdge.find(aBaseEdge)->second[0];
	}
	TopoDS_Vertex v1 = TopExp::FirstVertex(aBaseEdge);  // ��ȡ���
	TopoDS_Vertex v2 = TopExp::LastVertex(aBaseEdge);   // ��ȡ�յ�

	gp_Pnt baseStart = BRep_Tool::Pnt(v1);  // ��ȡ�������
	gp_Pnt baseEnd = BRep_Tool::Pnt(v2);

	// ����ģ���е����б�
	for (auto item : mapEdges) {
		TopoDS_Edge currentEdge = item.second;//�ɶ�ά
		if (m_oldMapNewEdge.find(currentEdge) != m_oldMapNewEdge.end())
		{
			currentEdge = m_oldMapNewEdge.find(currentEdge)->second[0];
		}
		TopoDS_Vertex w1 = TopExp::FirstVertex(currentEdge);  // ��ȡ���
		TopoDS_Vertex w2 = TopExp::LastVertex(currentEdge);
		gp_Pnt startPnt = BRep_Tool::Pnt(w1);
		gp_Pnt endPnt = BRep_Tool::Pnt(w2);

		// �жϵ�ǰ�ߵ������˵�ֱ��ڻ�׼�ߵ���һ��
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
		// ��������˵㶼��ͬһ��
		if ((sideStart >= 0 && sideEnd > 0) || ((sideStart > 0 && sideEnd >= 0))) {
			rightEdges.push_back(item.second);  // ���Ҳ�
		}
		else if ((sideStart <= 0 && sideEnd < 0) || ((sideStart < 0 && sideEnd <= 0))) {
			leftEdges.push_back(item.second); // �����
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
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge);  // ��ȡ���
	TopoDS_Vertex v2 = TopExp::LastVertex(edge);   // ��ȡ�յ�
	gp_Vec translationVec = translation.TranslationPart();
	gp_Pnt p1 = BRep_Tool::Pnt(v1);  // ��ȡ�������
	gp_Pnt p2 = BRep_Tool::Pnt(v2);
	// 5. �Աߵ������˵�Ӧ��ƽ��
	p1.Transform(translation);  // �ƶ����
	p2.Transform(translation);  // �ƶ��յ�

	// 6. ʹ���ƶ���ĵ㹹���µı�
	TopoDS_Edge newEdge = BRepBuilderAPI_MakeEdge(p1, p2);

	return newEdge;
}
//����ʱ�������������ƶ���,�ü���
TopoDS_Edge SheetFalttenMove::calRetractTranslate(const TopoDS_Edge& theEdge,const TopoDS_Face & theFace, const double theDistance ,double theAngle) {
	// 1. ��ȡ�ߵ������˵�
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
			TopoDS_Vertex w1 = TopExp::FirstVertex(a2dNewEdge);  // ��ȡ���
			TopoDS_Vertex w2 = TopExp::LastVertex(a2dNewEdge);   // ��ȡ�յ�

			p1 = BRep_Tool::Pnt(w1);  // ��ȡ�������
			p2 = BRep_Tool::Pnt(w2);
		}
		TopoDS_Vertex v1 = TopExp::FirstVertex(a2dBaseEdge);  // ��ȡ���
		TopoDS_Vertex v2 = TopExp::LastVertex(a2dBaseEdge);   // ��ȡ�յ�

		gp_Pnt baseStart = BRep_Tool::Pnt(v1);  // ��ȡ�������
		gp_Pnt baseEnd = BRep_Tool::Pnt(v2);  // ��ȡ�յ�����


		gp_Vec normalVec;


		for (TopExp_Explorer explorer(theFace, TopAbs_EDGE); explorer.More(); explorer.Next()) {
			const TopoDS_Edge& currentEdge = TopoDS::Edge(explorer.Current());
			if (!findMap_EdgeAndEdge_ToEdge(m_ThreeToTwoEdge, currentEdge, a2dTarEdge))
			{
				continue;
			}
			TopoDS_Vertex w1 = TopExp::FirstVertex(a2dTarEdge);  // ��ȡ���
			TopoDS_Vertex w2 = TopExp::LastVertex(a2dTarEdge);
			gp_Pnt startPnt = BRep_Tool::Pnt(w1);
			gp_Pnt endPnt = BRep_Tool::Pnt(w2);

			// �жϵ�ǰ�ߵ������˵�ֱ��ڻ�׼�ߵ���һ��
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
		//���
		processFlotInVec(normalVec);
		// 4. ����ƽ�Ʊ任����
		//gp_Vec translationVec = normalVec.Normalized() * distance;  // ��������λ��������ƽ�ƾ���
		gp_Vec translationVec = normalVec.Normalized() * theDistance;


		translation.SetTranslation(translationVec);  // ����ƽ�ƾ���

		// 5. �Աߵ������˵�Ӧ��ƽ��
		p1.Transform(translation);  // �ƶ����
		p2.Transform(translation);  // �ƶ��յ�

		// 6. ʹ���ƶ���ĵ㹹���µı�
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
	return newEdge;  // �����µı�
}
//�������������ƶ���
void SheetFalttenMove::calTranslate(const double distance, gp_Trsf& translation ,const double & angle) {
	gp_Vec vec = m_baseVec;
	//gp_Vec translationVec = normalVec.Normalized() * distance;  // ��������λ��������ƽ�ƾ���
	if (angle > M_PI)
	{
		vec = -m_baseVec;
	}
	gp_Vec translationVec = vec.Normalized() * distance;
	translation.SetTranslation(translationVec);  // ����ƽ�ƾ���
	return; // �����µı�
}

//�����м���
TopoDS_Edge midleLine(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2)
{
	// ��ȡ��һ���ߵ������˵�
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge1);  // ��ȡ���
	TopoDS_Vertex v2 = TopExp::LastVertex(edge1);
	gp_Pnt startPnt1 = BRep_Tool::Pnt(v1);
	gp_Pnt endPnt1 = BRep_Tool::Pnt(v2);

	// ��ȡ�ڶ����ߵ������˵�
	v1 = TopExp::FirstVertex(edge2);
	v2 = TopExp::LastVertex(edge2);
	gp_Pnt startPnt2 = BRep_Tool::Pnt(v1);
	gp_Pnt endPnt2 = BRep_Tool::Pnt(v2);

	// ���������е�
	gp_Pnt midStart((startPnt1.X() + startPnt2.X()) / 2.0,
		(startPnt1.Y() + startPnt2.Y()) / 2.0,
		(startPnt1.Z() + startPnt2.Z()) / 2.0);

	// �����յ���е�
	gp_Pnt midEnd((endPnt1.X() + endPnt2.X()) / 2.0,
		(endPnt1.Y() + endPnt2.Y()) / 2.0,
		(endPnt1.Z() + endPnt2.Z()) / 2.0);

	// �����м���
	TopoDS_Edge middleEdge = BRepBuilderAPI_MakeEdge(midStart, midEnd);

	return middleEdge;

}
//�ӳ������㣬���������⴦
bool SheetFalttenMove::TrimEdgesAtIntersection1(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection) {
	// ��ȡ��һ���߶ε������յ�
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(theEdge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(theEdge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// ��ȡ�ڶ����߶ε������յ�
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(theEdge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(theEdge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);
	// ���������߶εķ�������
	gp_Vec d1(p1_1, p1_2); // ��һ���߶εķ�������
	gp_Vec d2(p2_1, p2_2); // �ڶ����߶εķ�������

	gp_Vec crossProduct = d1.Crossed(d2);

	// �ж��Ƿ�ƽ��
	if (crossProduct.SquareMagnitude() < 1e-12) {
		return false; // ƽ�У��޽���
	}
	gp_Vec p1p3(p1_1, p2_1);
	// ���� t �� u
	double t = (p1p3.Crossed(d2)).Dot(crossProduct) / crossProduct.SquareMagnitude();
	double u = (p1p3.Crossed(d1)).Dot(crossProduct) / crossProduct.SquareMagnitude();

	// ���㽻������
	theIntersection = p1_1.Translated(t * d1);

	//if (theIntersection.IsEqual(p1_1, 1e-6) || theIntersection.IsEqual(p1_2, 1e-6))
	{
		// ���ݾ���ü�ÿ���߶ε������յ㣬ʹ�����ڽ��㴦���
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

//�ӳ������㣬������base�߶���
bool TrimEdgesAtIntersection(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection) {
	// ��ȡ��һ���߶ε������յ�
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(theEdge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(theEdge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// ��ȡ�ڶ����߶ε������յ�
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(theEdge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(theEdge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);
	// ���������߶εķ�������
	gp_Vec d1(p1_1, p1_2); // ��һ���߶εķ�������
	gp_Vec d2(p2_1, p2_2); // �ڶ����߶εķ�������

	gp_Vec crossProduct = d1.Crossed(d2);

	// �ж��Ƿ�ƽ��
	if (crossProduct.SquareMagnitude() < 1e-12) {
		return false; // ƽ�У��޽���
	}
	gp_Vec p1p3(p1_1, p2_1);
	// ���� t �� u
	double t = (p1p3.Crossed(d2)).Dot(crossProduct) / crossProduct.SquareMagnitude();
	double u = (p1p3.Crossed(d1)).Dot(crossProduct) / crossProduct.SquareMagnitude();

	// ���㽻������
	theIntersection = p1_1.Translated(t * d1);
	
	if (theIntersection.IsEqual(p1_1, 1e-6) || theIntersection.IsEqual(p1_2, 1e-6))
	{
		// ���ݾ���ü�ÿ���߶ε������յ㣬ʹ�����ڽ��㴦���
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
	
	

	// ���ݾ���ü�ÿ���߶ε������յ㣬ʹ�����ڽ��㴦���
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

//�ж������߶��ཻ
bool EdgeIntersect(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2)
{
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge1);  // ��ȡ���
	TopoDS_Vertex v2 = TopExp::LastVertex(edge1);
	gp_Pnt p1 = BRep_Tool::Pnt(v1);
	gp_Pnt p2 = BRep_Tool::Pnt(v2);

	TopoDS_Vertex w1 = TopExp::FirstVertex(edge2);  // ��ȡ���
	TopoDS_Vertex w2 = TopExp::LastVertex(edge2);
	gp_Pnt p3 = BRep_Tool::Pnt(w1);
	gp_Pnt p4 = BRep_Tool::Pnt(w2);
	gp_Vec d1(p1, p2); // ��һ���߶εķ�������
	gp_Vec d2(p3, p4); // �ڶ����߶εķ�������
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

//�ҵ�һ������Ľ���
bool SheetFalttenMove::relationEdge(TopoDS_Edge edge)
{
	vector<TopoDS_Edge> intersectionLines;
	vector<TopoDS_Edge> lines;
	auto faceIt = m_MapEdgeAdjFaces.find(edge);
	for (auto it : faceIt->second)
	{


		for (TopExp_Explorer edgeExplorer(it, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
		{

			TopoDS_Edge otherEdge = TopoDS::Edge(edgeExplorer.Current());//��ά����
			if (otherEdge != edge)
			{
				gp_Pnt intersection1;
				TopoDS_Edge twoEdge = m_ThreeToTwoEdge.find(otherEdge)->second;//�ɶ�ά
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

			TrimEdgesAtIntersection1(newbaseEdge, newInterseEdge, intersection);//�ӳ��ߵ�����
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

			TrimEdgesAtIntersection1(newbaseEdge, newInterseEdge, intersection);//�ӳ��ߵ�����
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
			
			TrimEdgesAtIntersection(newbaseEdge, newInterseEdge, intersection);//�ӳ��ߵ�����
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
// ������������н�
bool SheetFalttenMove::CalculateAngleBetweenFaces(const TopoDS_Face& theBaseFace,const TopoDS_Face &theTarFace, double& angle) {

	Handle(Geom_Surface) surface1 = BRep_Tool::Surface(theBaseFace);
	Handle(Geom_Surface) surface2 = BRep_Tool::Surface(theTarFace);
	// ���� surface1 �� surface2 �� Geom_Surface ���͵�����ƽ��
	GeomLProp_SLProps props1(surface1, 0.5, 0.5, 1, Precision::Confusion());
	GeomLProp_SLProps props2(surface2, 0.5, 0.5, 1, Precision::Confusion());

	// ��ȡ����ƽ��ķ�������
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
	// ����нǣ����ȣ�
	angle = normal1.Angle(normal2);

	// �ж�����Face�����ۻ��Ƿ���

	// �������ģ�����ͶӰƽ��plane
	GProp_GProps gpropsBase;
	BRepGProp::SurfaceProperties(theBaseFace, gpropsBase);
	gp_Pnt centerBase = gpropsBase.CentreOfMass();

	GProp_GProps gpropsTarget;
	BRepGProp::SurfaceProperties(theTarFace, gpropsTarget);
	gp_Pnt centerTarget = gpropsTarget.CentreOfMass();

	gp_Pln plane(centerBase, crossNormal);

	//���շ��߷�������ƽ�ƣ�����һ���µĵ�
	// �趨ƽ�ƾ���
	Standard_Real distance = 100.0;

	gp_Vec transVecBase = distance * gp_Vec(normal1);
	gp_Pnt centerBaseTrs = centerBase.Translated(transVecBase);

	gp_Vec transVecTarget = distance * gp_Vec(normal2);
	gp_Pnt centerTargetTrs = centerTarget.Translated(transVecTarget);

	//���ĸ���ͶӰ��plane��
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
// ������������н�
bool SheetFalttenMove::CalculateAngleBetweenFaces(const vector<TopoDS_Face>& face, double& angle) {

	Handle(Geom_Surface) surface1 = BRep_Tool::Surface(face[0]);
	Handle(Geom_Surface) surface2 = BRep_Tool::Surface(face[1]);
	// ���� surface1 �� surface2 �� Geom_Surface ���͵�����ƽ��
	GeomLProp_SLProps props1(surface1, 0.5, 0.5, 1, Precision::Confusion());
	GeomLProp_SLProps props2(surface2, 0.5, 0.5, 1, Precision::Confusion());

	// ��ȡ����ƽ��ķ�������
	gp_Dir normal1 = props1.Normal();
	gp_Dir normal2 = props2.Normal();
	gp_Dir crossNormal = normal1.Crossed(normal2);
	if (gp_Vec(crossNormal).Magnitude() < gp::Resolution())
	{
		return false;
	}
	// ����нǣ����ȣ�
	angle = normal1.Angle(normal2);

	// �ж�����Face�����ۻ��Ƿ���

	// �������ģ�����ͶӰƽ��plane
	GProp_GProps gpropsBase;
	BRepGProp::SurfaceProperties(face[0], gpropsBase);
	gp_Pnt centerBase = gpropsBase.CentreOfMass();

	GProp_GProps gpropsTarget;
	BRepGProp::SurfaceProperties(face[1], gpropsTarget);
	gp_Pnt centerTarget = gpropsTarget.CentreOfMass();

	gp_Pln plane(centerBase, crossNormal);

	//���շ��߷�������ƽ�ƣ�����һ���µĵ�
	// �趨ƽ�ƾ���
	Standard_Real distance = 100.0;

	gp_Vec transVecBase = distance * gp_Vec(normal1);
	gp_Pnt centerBaseTrs = centerBase.Translated(transVecBase);

	gp_Vec transVecTarget = distance * gp_Vec(normal2);
	gp_Pnt centerTargetTrs = centerTarget.Translated(transVecTarget);

	//���ĸ���ͶӰ��plane��
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
	map<TopoDS_Edge, TopoDS_Edge> aResult;//��ά�ı�
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

	makeLapelMap();//�۱�
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
	for (auto it : theMoveEdge)//�������ߵ���
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
	
		TopoDS_Edge baseEdge = m_ThreeToTwoEdge.find(edge)->second;//�ɶ�ά
		FindEdgesOnBothSides(m_ThreeToTwoEdge, baseEdge, leftEdges, rightEdges, colines, otherlines);//��������װ�Ķ��Ǿɶ�ά,����Ų
		/*if (angle > M_PI)
		{
			m_baseVec = -m_baseVec;
		}*/
		double moveDis = 2.;
		calTranslate(moveDis, translation_slot, angle);
		calTranslate(moveDis, translation ,angle);//ȷ���ƶ�����ͽǶ�
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
		for (auto it : colines)//�������ߵ���
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
		aCompound.Nullify(); // �� compound ��Ϊ��Ч״̬
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
//�˵��Ƿ���ͬ
bool SheetFalttenMove::AreEdgesSameEndpoints(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2) {
	// ��ȡ��һ���ߵ������˵�
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// ��ȡ�ڶ����ߵ������˵�
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);

	// �ж������ߵĶ˵��Ƿ���ͬ������˳��
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