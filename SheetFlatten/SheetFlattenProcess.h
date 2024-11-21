#ifndef SHEETFLATTENPROCESS_H
#define SHEETFLATTENPROCESS_H


#include <BRepBuilderAPI_MakeEdge.hxx>
#include <GProp_GProps.hxx>
#include <BRepGProp.hxx>
#include <GeomLProp_SLProps.hxx>
#include <ProjLib.hxx>
#include <IntAna2d_AnaIntersection.hxx>
#include<TopExp.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <Quantity_Color.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <BRep_Builder.hxx>
#include <GeomAPI_ExtremaCurveCurve.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <TopoDS_Shape.hxx>

#include "SheetFlattenEdgeData.h"


#include <iostream>
#include <vector>
#include<map>
#include<set>

using namespace std;

struct moveData {
	int adjEdgeDataIndex;
	double distance = -1.;
	gp_Vec moveVec;
};


class SheetFlattenProcess
{
public:
	SheetFlattenProcess();
	~SheetFlattenProcess();

private:
	void moveSplitEdge();
	void processEdges_WrapAngle();
	void processSplitEdge();
	void QuirySplitEdge();
	void processSameFaceEdgeRelation();
	bool isOverlap(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2);
	gp_Pnt findOtherPoint(const TopoDS_Edge& theEdge, const gp_Pnt& thePoint);
	bool getAdjFace(TopoDS_Edge edge, vector<TopoDS_Face>& result);
	bool relationEdge(TopoDS_Edge edge);
	void allReation2dEdge();
	TopoDS_Edge midleLine(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2);
	void FindEdgesOnBothSides(vector<SheetFlattenEdgeData>& edges, SheetFlattenEdgeData& baseEdge,
		std::vector<TopoDS_Edge>& leftEdges, std::vector<TopoDS_Edge>& rightEdges, std::vector<TopoDS_Edge>& colines, std::vector<TopoDS_Edge>& otherlines);
	void calTranslate(const double distance, gp_Trsf& translation, const double& angle);
	TopoDS_Edge calRetractTranslate(SheetFlattenEdgeData &theEdge, const TopoDS_Face& theFace, const double theDistance, double theAngle);
	TopoDS_Edge TranslateEdge(const TopoDS_Edge& edge, const gp_Trsf& translation);
	bool CalculateAngleBetweenFaces(const vector<TopoDS_Face>& face, double& angle);
	bool CalculateAngleBetweenFaces(const TopoDS_Face& theBaseFace, const TopoDS_Face& theTarFace, double& angle);
	void moveEdge(vector<SheetFlattenEdgeData>& edges, SheetFlattenEdgeData& edge);
	void processOverlap();
	TopoDS_Edge findAdjEdge(const TopoDS_Edge &baseEdge, const vector<TopoDS_Edge> &interEdges, const gp_Pnt &basePoint);
	void generateMidleEdge(const vector<TopoDS_Edge>& theMoveEdge, const gp_Trsf& translation, const gp_Trsf& theTranslation_slot);
	void generateNewEdge(const vector<TopoDS_Edge>& theMoveEdge, const gp_Trsf& translation, bool theMove);
	void generateSoltEdge(const TopoDS_Edge& theEdge);
	void SheetFlattenProcess::gerateNegativeBend(SheetFlattenEdgeData*& pEdgedata);
	void generateOverlapEdge(const TopoDS_Edge& theEdge);
	bool SegmentsOverlap(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, TopoDS_Edge& newEdge1, TopoDS_Edge& newEdge2,
		gp_Pnt& newPoint1, gp_Pnt& newPoint2, gp_Pnt& startPoint1, gp_Pnt& startPoint2, gp_Vec& directionVec1, gp_Vec& directionVec2);
	void processFlotInVec(gp_Vec& normalVec);
	gp_Vec CalMoveVector(const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p);
	void calTranslate(SheetFlattenEdgeData& theEdge, const TopoDS_Face& theFace, gp_Vec& theVec);
	int GetPointSide1(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point);
	int GetPointSide(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point);
	void allReation2dNewEdge();
	void findSameEdge(map<TopoDS_Edge, TopoDS_Edge>& result);
	bool judgeless(double x, double y);
	bool EdgeIntersect(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2);
	bool TrimEdgesAtIntersection1(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection);
	bool TrimEdgesAtIntersection(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection);
	TopoDS_Edge TranslatePositiveEdge(const TopoDS_Edge& edge, const gp_Trsf& translation, TopoDS_Edge& theNewEdge, TopoDS_Edge& theNewEdge2);
	gp_Pnt GetPerpendicularFootPoint(const gp_Pnt& point, const TopoDS_Edge& edge);
	bool quiryNewEdge_2d(TopoDS_Edge& baseEdge, TopoDS_Edge& tarEdge);
	bool quiryNewEdge_2d(TopoDS_Edge& baseEdge, vector<TopoDS_Edge>& tarEdge);
	bool quiryEdgeData(const TopoDS_Edge& baseEdge, SheetFlattenEdgeData*& data);
	void changeMapIdex(const TopoDS_Edge& baseEdge, const TopoDS_Edge& tarEdge);
	bool quiry3dEdgeData(const TopoDS_Edge& baseEdge, SheetFlattenEdgeData*& data);
	void openSlot();
	void wrapAngle();
	void makeLapelMap();
	void processOutline();
	void translateLapel(const double theDistance);
	bool isSameFace(const vector<TopoDS_Face>& theLeftFaces, const vector<TopoDS_Face>& theRightFaces);
	bool AreEdgesSameEndpoints(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2);
	bool IsEdgesAtIntersection(TopoDS_Edge& edge1, TopoDS_Edge& edge2, gp_Pnt& intersection);
	bool findMap_EdgeAndVectorFace_ToFace(const map < TopoDS_Edge, vector<TopoDS_Face>>& theMap, const TopoDS_Edge& theEdge, TopoDS_Face& theFace);
	bool findMap_EdgeAndVEctorFace_ToVectorFace(const map<TopoDS_Edge, vector<TopoDS_Face>>& theMap, const TopoDS_Edge& theEdge, vector<TopoDS_Face>& theFaces);
	bool findMap_EdgeAndEdge_ToEdge(const map<TopoDS_Edge, TopoDS_Edge>& theMapEdges, const TopoDS_Edge& theTarEdge_2d, TopoDS_Edge& theTarEdge_3d);
	bool findMap_EdgeAndVector_ToEdge(const map<TopoDS_Edge, vector<TopoDS_Edge>>& MapEdge, const TopoDS_Edge& theOldEdge, TopoDS_Edge& theNewEdge);
	bool findMap_EdgeAndVector_ToVector(const map<TopoDS_Edge, vector<TopoDS_Edge>>& MapEdge, const TopoDS_Edge& theOldEdge, vector<TopoDS_Edge>& theVector);
	void calTranslateOriention(const TopoDS_Edge& theEdge, const gp_Pnt& thePoint, gp_Trsf& theTranslation, const double theDistance);
public:
	void check();
	void generate(TopoDS_Compound& aCompound);
	void generateEdge(const TopoDS_Edge& theEdge);
	void setEdgeData(const vector<vector<SheetFlattenEdgeData>>& EdgeData);
	void setOldEdgeIndex(const map<TopoDS_Edge, pair<int, int>>& index);
	void set3dEdgeIndex(const map<TopoDS_Edge, pair<int, int>>& mapindex);
	//void getRefDir(const TopoDS_Edge& edge);
public:
	TopTools_IndexedMapOfShape faceMap;
	vector<vector<SheetFlattenEdgeData>> m_EdgeData;
	map<int, vector<moveData>> m_adjGroupIndex;
	map<TopoDS_Edge, pair<int, int>> m_oldEdgeIndex;
	map<TopoDS_Edge, pair<int, int>> m_3dEdgeIndex;
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
	bool m_isNegativeBend;
	vector<TopoDS_Edge> m_outlindeEdges;
};

SheetFlattenProcess::SheetFlattenProcess()
{
}

SheetFlattenProcess::~SheetFlattenProcess()
{
}

void SheetFlattenProcess::changeMapIdex(const TopoDS_Edge& oldEdge, const TopoDS_Edge& newEdge)
{
	auto it = m_oldEdgeIndex.find(oldEdge); // �ҵ���Ϊ 2 ��Ԫ��
	if (it != m_oldEdgeIndex.end()) {
		m_oldEdgeIndex[newEdge] = it->second; // �����¼�ֵ�ԣ�ֵ���ֲ���
		m_oldEdgeIndex.erase(it);       // ɾ���ɼ�ֵ��
	}
}


bool SheetFlattenProcess::quiryEdgeData(const TopoDS_Edge & baseEdge,SheetFlattenEdgeData*& data)
{
	if (m_oldEdgeIndex.find(baseEdge) != m_oldEdgeIndex.end())
	{
		data = &m_EdgeData[m_oldEdgeIndex.find(baseEdge)->second.first][m_oldEdgeIndex.find(baseEdge)->second.second];
		return true;
	}
	return false;
}

bool SheetFlattenProcess::quiry3dEdgeData(const TopoDS_Edge& baseEdge, SheetFlattenEdgeData*& data)
{
	if (m_3dEdgeIndex.find(baseEdge) != m_3dEdgeIndex.end())
	{
		data = &m_EdgeData[m_3dEdgeIndex.find(baseEdge)->second.first][m_3dEdgeIndex.find(baseEdge)->second.second];
		return true;
	}
	return false;
}
void SheetFlattenProcess::QuirySplitEdge()
{
	for (int i = 0; i < m_EdgeData.size()-1; ++i)
	{
		vector<SheetFlattenEdgeData> aBaseEdgeData = m_EdgeData[i];
		vector<SheetFlattenEdgeData> aTarEdgeData = m_EdgeData[i+1];
		for (int j = 0; j<aBaseEdgeData.size(); ++j)
		{
			TopoDS_Edge aBaseEdge_3d = aBaseEdgeData[j].getEdge_3d();
			for (int t = 0; t < aTarEdgeData.size(); ++t)
			{
				TopoDS_Edge aTarEdge_3d = aTarEdgeData[t].getEdge_3d();
				if (AreEdgesSameEndpoints(aTarEdge_3d, aBaseEdge_3d))
				{
					double aAngle = 0;
					CalculateAngleBetweenFaces(aBaseEdgeData[j].getVector_face()[0], aTarEdgeData[t].getVector_face()[0], aAngle);
					m_EdgeData[i][j].setSplitEdge(true);
					m_EdgeData[i][j].setSplitIndex(make_pair(i + 1, t));
					m_EdgeData[i][j].setAngle(aAngle);
					m_EdgeData[i + 1][t].setSplitEdge(true);
					m_EdgeData[i + 1][t].setSplitIndex(make_pair(i, j));
					m_EdgeData[i + 1][t].setAngle(aAngle);
				}
			}
		}
	}
}
//void SetEdgeColor(const TDF_Label& shapeLabel, const Quantity_Color& color) {
//	Handle(XCAFDoc_ColorTool) colorTool = XCAFDoc_DocumentTool::ColorTool(shapeLabel);
//	if (!colorTool.IsNull()) {
//		// ʹ��XCAFDoc_ColorType::XCAFDoc_ColorCurves�趨�ߵ���ɫ
//		colorTool->SetColor(shapeLabel, color, XCAFDoc_ColorType::XCAFDoc_ColorCurv);
//	}
//}
void SheetFlattenProcess::setEdgeData(const vector<vector<SheetFlattenEdgeData>>& EdgeData)
{
	m_EdgeData = EdgeData;
}
void SheetFlattenProcess::setOldEdgeIndex(const map<TopoDS_Edge, pair<int, int>>& mapindex)
{
	m_oldEdgeIndex = mapindex;
}
void SheetFlattenProcess::set3dEdgeIndex(const map<TopoDS_Edge, pair<int, int>>& mapindex)
{
	m_3dEdgeIndex = mapindex;
}
void SheetFlattenProcess::calTranslateOriention(const TopoDS_Edge& theEdge, const gp_Pnt& thePoint, gp_Trsf& theTranslation, const double theDistance)
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
gp_Pnt SheetFlattenProcess::findOtherPoint(const TopoDS_Edge& theEdge, const gp_Pnt& thePoint)
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
bool SheetFlattenProcess::quiryNewEdge_2d(TopoDS_Edge & baseEdge,TopoDS_Edge & tarEdge)
{
	int i = 0, j = 0;
	if (m_oldEdgeIndex.find(baseEdge) != m_oldEdgeIndex.end())
	{
		i = m_oldEdgeIndex.find(baseEdge)->second.first;
		j = m_oldEdgeIndex.find(baseEdge)->second.second;
		if (m_EdgeData[i][j].getNewEdge_2d(tarEdge))
		{
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}

bool SheetFlattenProcess::quiryNewEdge_2d(TopoDS_Edge& baseEdge, vector<TopoDS_Edge>& tarEdge)
{
	int i = 0, j = 0;
	if (m_oldEdgeIndex.find(baseEdge) != m_oldEdgeIndex.end())
	{
		i = m_oldEdgeIndex.find(baseEdge)->second.first;
		j = m_oldEdgeIndex.find(baseEdge)->second.second;
		if (m_EdgeData[i][j].getVector_newEdge_2d(tarEdge))
		{
			return true;
		}
		else {
			return false;
		}
	}
	return false;
}
void SheetFlattenProcess::translateLapel(const double theDistance)
{
	gp_Pnt aInterPoint, aOldInterPoint, aOldPoint1, aOldPoint2;
	TopoDS_Edge aBaseEdge, aLapelEdge1, aLapelEdge2, aLapelNewEdge1, aLapelNewEdge2, aTranslateLaterEdge1, aTranslateLaterEdge2;
	TopoDS_Edge aNewLapelInterEdge1, aNewLapelInterEdge2, aOldLapelInterEdge1, aOldLapelInterEdge2;;
	vector<TopoDS_Edge> aInterLapeEdges1, aInterLapeEdges2,aLapeEdges;
	gp_Trsf aTranslation;
	SheetFlattenEdgeData* pEdgedata1, * pEdgedata2,*pInterEdgedata1,*pInterEdgedata2;

	for (auto &elem : m_EdgeData)
	{
		for (auto &item : elem)
		{
			if (!item.getVector_LapelEdges(aLapeEdges))
			{
				continue;
			}
			aBaseEdge = item.getOldEdge_2d();
			aLapelEdge1 = aLapeEdges[0];//�϶�ά
			aLapelEdge2 = aLapeEdges[1];
			quiryEdgeData(aLapelEdge1, pEdgedata1);
			quiryEdgeData(aLapelEdge2, pEdgedata2);
			if (pEdgedata1->isOverlapeEdge() || pEdgedata2->isOverlapeEdge())
			{
				continue;
			}
			IsEdgesAtIntersection(aLapelEdge1, aLapelEdge2, aOldInterPoint);

			aOldPoint1 = findOtherPoint(aLapelEdge1, aOldInterPoint);
			aOldPoint2 = findOtherPoint(aLapelEdge2, aOldInterPoint);
			

			aOldLapelInterEdge1 = findAdjEdge(aLapelEdge1, pEdgedata1->getVector_interseEdge_2d(), aOldPoint1);//�϶�ά
			aOldLapelInterEdge2 = findAdjEdge(aLapelEdge2, pEdgedata2->getVector_interseEdge_2d(), aOldPoint2);

			quiryEdgeData(aOldLapelInterEdge1, pInterEdgedata1);
			pInterEdgedata1->getNewEdge_2d(aNewLapelInterEdge1);

			quiryEdgeData(aOldLapelInterEdge2, pInterEdgedata2);
			pInterEdgedata2->getNewEdge_2d(aNewLapelInterEdge2);


			pEdgedata1->getNewEdge_2d(aLapelNewEdge1);
			pEdgedata2->getNewEdge_2d(aLapelNewEdge2);


			IsEdgesAtIntersection(aBaseEdge, aLapelEdge1, aInterPoint);
			calTranslateOriention(aBaseEdge, aInterPoint, aTranslation, theDistance);
			aTranslateLaterEdge1 = TranslateEdge(aLapelNewEdge1, aTranslation);
			aTranslateLaterEdge2 = TranslateEdge(aLapelNewEdge2, aTranslation);


			TrimEdgesAtIntersection1(aTranslateLaterEdge1, aTranslateLaterEdge2, aInterPoint);

			TrimEdgesAtIntersection1(aTranslateLaterEdge1, aNewLapelInterEdge1, aInterPoint);
			TrimEdgesAtIntersection1(aTranslateLaterEdge2, aNewLapelInterEdge2, aInterPoint);

			pEdgedata1->insertEdgeTo_new2d(aTranslateLaterEdge1);
			pEdgedata2->insertEdgeTo_new2d(aTranslateLaterEdge2);
			vector<TopoDS_Edge> aEdges;

			if (pInterEdgedata1->getVector_newEdge_2d(aEdges))
			{
				if (aEdges.size() > 1)
				{
					aEdges[aEdges.size() - 1] = aNewLapelInterEdge1;
					pInterEdgedata1->setVector_newEdge_2d(aEdges);

				}
				else
				{
					pInterEdgedata1->insertEdgeTo_new2d(aNewLapelInterEdge1);
				}
			}

			if (pInterEdgedata2->getVector_newEdge_2d(aEdges))
			{
				if (aEdges.size() > 1)
				{
					aEdges[aEdges.size() - 1] = aNewLapelInterEdge2;
					pInterEdgedata2->setVector_newEdge_2d(aEdges);

				}
				else
				{
					pInterEdgedata2->insertEdgeTo_new2d(aNewLapelInterEdge2);
				}
			}

			//m_oldMapNewEdge.find(aOldLapelInterEdge1)->second.emplace_back(aNewLapelInterEdge1);
			//m_oldMapNewEdge.find(aOldLapelInterEdge2)->second.emplace_back(aNewLapelInterEdge2);


		}
	}
}
bool SheetFlattenProcess::findMap_EdgeAndVector_ToVector(const map<TopoDS_Edge, vector<TopoDS_Edge>>& MapEdge, const TopoDS_Edge& theOldEdge, vector<TopoDS_Edge>& theVector)
{
	theVector.clear();
	if (MapEdge.find(theOldEdge) != MapEdge.end())
	{
		auto iter = MapEdge.find(theOldEdge);
		theVector = iter->second;
		return true;
	}
	return false;
}


bool SheetFlattenProcess::isSameFace(const vector<TopoDS_Face> &theLeftFaces, const vector<TopoDS_Face> &theRightFaces)
{
	set<TopoDS_Face> aFinishFaces;
	vector<bool> aResult;
	bool aTempResult = false;
	

		
			for (auto itLeft : theLeftFaces)
			{
				aFinishFaces.insert(itLeft);
				aTempResult = false;
				for (auto itRight : theRightFaces)
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
		
	return false;
	//return AreAllTrue(aResult);
}

void SheetFlattenProcess::makeLapelMap()
{
	TopoDS_Edge aBaseEdge, aBaseEdge3d, aLeftEdge, aRightEdge;
	vector<TopoDS_Face> aLeftFaces, aRightFaces, aBaseFaces;
	vector<TopoDS_Edge> aVectorResult;
	vector<TopoDS_Edge> aLapelEdges;
	set<TopoDS_Edge> aFinishEdges;

	for (auto &elem : m_EdgeData)
	{
		for (auto &it : elem)
		{
			if (it.getVector_newEdge_2d().size() >= 3)
			{
				aBaseEdge = it.getEdge_3d();
				aVectorResult = it.getVector_interseEdge_2d();
				aBaseFaces = it.getVector_face();
				//if (findMap_EdgeAndVector_ToVector(m_MapInterseLines, aBaseEdge, aVectorResult))
				{
					for (int i = 0; i < aVectorResult.size(); i++)
					{
						SheetFlattenEdgeData* pLeftEdgedata,*pRightEdgedata;
						quiryEdgeData(aVectorResult[i], pLeftEdgedata);
						aLeftEdge = pLeftEdgedata->getEdge_3d();
						aLeftFaces = pLeftEdgedata->getVector_face();
						//findMap_EdgeAndEdge_ToEdge(m_TwoToThreeEdge, aVectorResult[i], aLeftEdge);
						for (int j = 0; j < aVectorResult.size(); j++)
						{
							if (j != i && aFinishEdges.find(aVectorResult[i]) == aFinishEdges.end())
							{
								quiryEdgeData(aVectorResult[j], pRightEdgedata);
								aRightEdge = pRightEdgedata->getEdge_3d();
								aRightFaces = pRightEdgedata->getVector_face();
								if (AreEdgesSameEndpoints(aLeftEdge, aRightEdge))
								{
									if (!isSameFace(aBaseFaces, aLeftFaces))
									{
										if (!isSameFace(aBaseFaces, aRightFaces))
										{
											aFinishEdges.insert(aVectorResult[i]);
											aLapelEdges.emplace_back(aVectorResult[i]);
											aLapelEdges.emplace_back(aVectorResult[j]);
											it.setVector_LapelEdges(aLapelEdges);
											//m_MapLapelEdges[aBaseEdge] = aLapelEdges;
											aLapelEdges.clear();
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}

}
void SheetFlattenProcess::openSlot()
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
bool SheetFlattenProcess::IsEdgesAtIntersection(TopoDS_Edge& edge1, TopoDS_Edge& edge2, gp_Pnt& intersection) {
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
void SheetFlattenProcess::processFlotInVec(gp_Vec& normalVec)
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
bool SheetFlattenProcess::isOverlap(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2)
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
bool SheetFlattenProcess::SegmentsOverlap(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, TopoDS_Edge& newEdge1, TopoDS_Edge& newEdge2,
	gp_Pnt& newPoint1, gp_Pnt& newPoint2, gp_Pnt& startPoint1, gp_Pnt& startPoint2, gp_Vec& directionVec1, gp_Vec& directionVec2) {
	gp_Pnt basePoint;
	if (AreEdgesSameEndpoints(edge1, edge2))
	{
		return false;
	}
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


//ͨ�������ҽ���
TopoDS_Edge SheetFlattenProcess::findAdjEdge(const TopoDS_Edge &baseEdge,const vector<TopoDS_Edge> &interEdges, const gp_Pnt &basePoint)
{
	
		for (auto it : interEdges)
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

bool SheetFlattenProcess::findMap_EdgeAndVector_ToEdge(const map<TopoDS_Edge, vector<TopoDS_Edge>>& MapEdge, const TopoDS_Edge& theOldEdge, TopoDS_Edge& theNewEdge)
{
	if (MapEdge.find(theOldEdge) != MapEdge.end())
	{
		auto iter = MapEdge.find(theOldEdge);
		theNewEdge = iter->second.back();
		//theNewEdge = iter->second[0];
		return true;
	}
	return false;
}


bool SheetFlattenProcess::findMap_EdgeAndEdge_ToEdge(const map<TopoDS_Edge, TopoDS_Edge>& theMapEdges, const TopoDS_Edge& theTarEdge_2d, TopoDS_Edge& theTarEdge_3d)
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

bool SheetFlattenProcess::findMap_EdgeAndVectorFace_ToFace(const map < TopoDS_Edge, vector<TopoDS_Face>>& theMap, const TopoDS_Edge& theEdge, TopoDS_Face& theFace)
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

bool SheetFlattenProcess::findMap_EdgeAndVEctorFace_ToVectorFace(const map<TopoDS_Edge, vector<TopoDS_Face>>& theMap, const TopoDS_Edge& theEdge, vector<TopoDS_Face>& theFaces)
{
	if (theMap.find(theEdge) != theMap.end())
	{
		theFaces = theMap.find(theEdge)->second;
		return true;
	}
	return false;
}
// ��������㵽ֱ�ߵĴ��㽻��
gp_Pnt SheetFlattenProcess::GetPerpendicularFootPoint(const gp_Pnt& point, const TopoDS_Edge& edge)
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
void SheetFlattenProcess::processOverlap()
{
	TopoDS_Edge tempEdge1, tempEdge2, oldTarEdge1_2d, oldTarEdge2_2d, newTarEdge1, newTarEdge2;
	gp_Vec aDirvec1, aDirvec2;
	gp_Pnt aInterPoint1, aInterPoint2, aStartPoint1, aStartPoint2, aNewPoint1, aNewPoint2;
	for (auto &elem : m_EdgeData)
	{
		for (auto &it : elem)
		{
			for (auto &item : elem)
			{
				TopoDS_Edge new1, new2;
				it.getNewEdge_2d(new1);
				item.getNewEdge_2d(new2);
				if (new1 != new2)
				{

					if (SegmentsOverlap(new1, new2, tempEdge1, tempEdge2, aInterPoint1, aInterPoint2, aStartPoint1, aStartPoint2, aDirvec1, aDirvec2))
					{
						TopoDS_Edge old2d1, old2d2;
						//old2d1 = it.getOldEdge_2d();
						oldTarEdge1_2d = findAdjEdge(it.getOldEdge_2d(),it.getVector_interseEdge_2d(), aInterPoint2);//��������
						oldTarEdge2_2d = findAdjEdge(item.getOldEdge_2d(), item.getVector_interseEdge_2d(), aInterPoint1);
						SheetFlattenEdgeData *aEdgedata1 = nullptr,*aEdgedata2 = nullptr;
						if (quiryEdgeData(oldTarEdge1_2d, aEdgedata1))
						{
							aEdgedata1->setOverlapeEdge(true);
						}
						if (quiryEdgeData(oldTarEdge2_2d, aEdgedata2))
						{
							aEdgedata2->setOverlapeEdge(true);
						}
						//m_overlapEdges.insert(oldTarEdge1_2d);
						//m_overlapEdges.insert(oldTarEdge2_2d);
						
						gp_Pnt aPoint1, aPoint2;
						IsEdgesAtIntersection(oldTarEdge1_2d, oldTarEdge2_2d, aPoint1);//�ҵ�����
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


						aEdgedata1->setOverLapEdge_Point(aPoint1);
						aEdgedata2->setOverLapEdge_Point(aPoint2);


						vector<TopoDS_Edge> aEdges;
						aEdges.emplace_back(tempEdge2);
						it.setVector_newEdge_2d(aEdges);
						//m_oldMapNewEdge[it.second] = aEdges;
						aEdges.clear();


						aEdges.emplace_back(tempEdge1);
						item.setVector_newEdge_2d(aEdges);
						//m_oldMapNewEdge[item.second] = aEdges;
						aEdges.clear();

						aEdges.emplace_back(newTarEdge1);
						aEdgedata1->setVector_newEdge_2d(aEdges);
						//m_oldMapNewEdge[oldTarEdge1_2d] = aEdges;
						aEdges.clear();

						aEdges.emplace_back(newTarEdge2);
						aEdgedata2->setVector_newEdge_2d(aEdges);
						//m_oldMapNewEdge[oldTarEdge2_2d] = aEdges;
						aEdges.clear();
					}
				}
			}
		}
	}



	//for (auto it : m_ThreeToTwoEdge)
	//{
	//	for (auto item : m_ThreeToTwoEdge)
	//	{
	//		TopoDS_Edge new1, new2;
	//		if (!findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, it.second, new1))
	//		{
	//			new1 = it.second;
	//		}
	//		if (!findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, item.second, new2))
	//		{
	//			new2 = item.second;
	//		}
	//		if (new1 != new2)
	//		{

	//			if (SegmentsOverlap(new1, new2, tempEdge1, tempEdge2, aInterPoint1, aInterPoint2, aStartPoint1, aStartPoint2, aDirvec1, aDirvec2))
	//			{
	//				oldTarEdge1_2d = findAdjEdge(it.second, aInterPoint2);//��������
	//				oldTarEdge2_2d = findAdjEdge(item.second, aInterPoint1);
	//				m_overlapEdges.insert(oldTarEdge1_2d);
	//				m_overlapEdges.insert(oldTarEdge2_2d);

	//				gp_Pnt aPoint1, aPoint2;
	//				IsEdgesAtIntersection(oldTarEdge1_2d, oldTarEdge2_2d, aPoint1);//�ҵ�����
	//				aPoint2 = aPoint1;
	//				// ���ŵ�λ���������ݸ����ľ�������λ������

	//				double distance = 3.;
	//				gp_Vec displacement1 = aDirvec1.Scaled(distance);
	//				// ����ʼ�� p1 ���ŷ��������ƶ��������µĵ�
	//				aPoint1 = aPoint1.Translated(displacement1);//�ƶ���Ľ���
	//				gp_Pnt vecticalPoint1 = GetPerpendicularFootPoint(aPoint1, tempEdge1);//���߽���
	//				newTarEdge1 = BRepBuilderAPI_MakeEdge(aPoint1, vecticalPoint1);
	//				tempEdge1 = BRepBuilderAPI_MakeEdge(aStartPoint1, vecticalPoint1);


	//				gp_Vec displacement2 = aDirvec2.Scaled(distance);
	//				// ����ʼ�� p1 ���ŷ��������ƶ��������µĵ�
	//				aPoint2 = aPoint2.Translated(displacement2);
	//				gp_Pnt vecticalPoint2 = GetPerpendicularFootPoint(aPoint2, tempEdge2);
	//				newTarEdge2 = BRepBuilderAPI_MakeEdge(aPoint2, vecticalPoint2);
	//				tempEdge2 = BRepBuilderAPI_MakeEdge(vecticalPoint2, aStartPoint2);


	//				vector<TopoDS_Edge> aEdges;
	//				aEdges.emplace_back(tempEdge2);
	//				m_oldMapNewEdge[it.second] = aEdges;
	//				aEdges.clear();


	//				aEdges.emplace_back(tempEdge1);
	//				m_oldMapNewEdge[item.second] = aEdges;
	//				aEdges.clear();

	//				aEdges.emplace_back(newTarEdge1);
	//				m_oldMapNewEdge[oldTarEdge1_2d] = aEdges;
	//				aEdges.clear();

	//				aEdges.emplace_back(newTarEdge2);
	//				m_oldMapNewEdge[oldTarEdge2_2d] = aEdges;
	//				aEdges.clear();
	//			}
	//		}
	//	}
	//}
}

int SheetFlattenProcess::GetPointSide1(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point) {
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
int SheetFlattenProcess::GetPointSide(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point) {
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
gp_Vec SheetFlattenProcess::CalMoveVector(const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p) {
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
void SheetFlattenProcess::FindEdgesOnBothSides(vector<SheetFlattenEdgeData>& edges, SheetFlattenEdgeData& baseEdge,
	std::vector<TopoDS_Edge>& leftEdges, std::vector<TopoDS_Edge>& rightEdges, std::vector<TopoDS_Edge>& colines, std::vector<TopoDS_Edge>& otherlines) {
	// ��ȡ��׼�ߵ������յ�
	TopoDS_Edge aBaseEdge = baseEdge.getOldEdge_2d();
	//baseEdge.getVector_newEdge_2d(aBaseEdge);
	TopoDS_Vertex v1 = TopExp::FirstVertex(aBaseEdge);  // ��ȡ���
	TopoDS_Vertex v2 = TopExp::LastVertex(aBaseEdge);   // ��ȡ�յ�

	gp_Pnt baseStart = BRep_Tool::Pnt(v1);  // ��ȡ�������
	gp_Pnt baseEnd = BRep_Tool::Pnt(v2);

	// ����ģ���е����б�
	for (auto &item : edges) {
		TopoDS_Edge currentEdge = item.getOldEdge_2d();//�ɶ�ά
		//item.getNewEdge_2d(currentEdge);
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
			if (vec1.IsEqual(vec2, 1e-4, 1e-4))
			{
				m_baseVec = CalMoveVector(baseStart, baseEnd, startPnt);
			}
		}
		sideStart = GetPointSide(baseStart, baseEnd, startPnt);
		sideEnd = GetPointSide(baseStart, baseEnd, endPnt);
		// ��������˵㶼��ͬһ��
		if ((sideStart >= 0 && sideEnd > 0) || ((sideStart > 0 && sideEnd >= 0))) {
			rightEdges.push_back(item.getOldEdge_2d());  // ���Ҳ�
		}
		else if ((sideStart <= 0 && sideEnd < 0) || ((sideStart < 0 && sideEnd <= 0))) {
			leftEdges.push_back(item.getOldEdge_2d()); // �����
		}
		else if (sideStart == 0 && sideEnd == 0)
		{
			colines.push_back(item.getOldEdge_2d());
		}
		else {
			otherlines.push_back(item.getOldEdge_2d());
		}
	}
}


TopoDS_Edge SheetFlattenProcess::TranslateEdge(const TopoDS_Edge& edge, const gp_Trsf& translation)
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

TopoDS_Edge SheetFlattenProcess::TranslatePositiveEdge(const TopoDS_Edge& edge, const gp_Trsf& translation, TopoDS_Edge& theNewEdge1, TopoDS_Edge& theNewEdge2)
{
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge);  // ��ȡ���
	TopoDS_Vertex v2 = TopExp::LastVertex(edge);   // ��ȡ�յ�
	gp_Vec translationVec = translation.TranslationPart();
	gp_Pnt p1 = BRep_Tool::Pnt(v1);  // ��ȡ�������
	gp_Pnt p2 = BRep_Tool::Pnt(v2);

	gp_Pnt old_p1 = p1;
	gp_Pnt old_p2 = p2;


	// 5. �Աߵ������˵�Ӧ��ƽ��
	p1.Transform(translation);  // �ƶ����
	p2.Transform(translation);  // �ƶ��յ�
	theNewEdge1 = BRepBuilderAPI_MakeEdge(old_p1, p1);
	theNewEdge2 = BRepBuilderAPI_MakeEdge(old_p2, p2);
	// 6. ʹ���ƶ���ĵ㹹���µı�
	TopoDS_Edge newEdge = BRepBuilderAPI_MakeEdge(p1, p2);

	return newEdge;
}

void SheetFlattenProcess::calTranslate(SheetFlattenEdgeData& theEdge, const TopoDS_Face& theFace,gp_Vec & theVec)
{
	gp_Trsf translation;
	TopoDS_Edge a2dBaseEdge, a2dTarEdge, a2dNewEdge, a2dInterEdge;
	TopoDS_Edge newEdge;
	gp_Pnt p1, p2, intersection;
	bool isReduct = true;
	a2dBaseEdge = theEdge.getOldEdge_2d();

	TopoDS_Vertex v1 = TopExp::FirstVertex(a2dBaseEdge);  // ��ȡ���
	TopoDS_Vertex v2 = TopExp::LastVertex(a2dBaseEdge);   // ��ȡ�յ�
	gp_Pnt baseStart = BRep_Tool::Pnt(v1);  // ��ȡ�������
	gp_Pnt baseEnd = BRep_Tool::Pnt(v2);  // ��ȡ�յ�����


	gp_Vec normalVec;

	for (TopExp_Explorer explorer(theFace, TopAbs_EDGE); explorer.More(); explorer.Next()) {
		const TopoDS_Edge& currentEdge = TopoDS::Edge(explorer.Current());
		SheetFlattenEdgeData* pCurrentEdgedata;
		quiry3dEdgeData(currentEdge, pCurrentEdgedata);
		a2dTarEdge = pCurrentEdgedata->getOldEdge_2d();
		if (a2dBaseEdge == a2dTarEdge)
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

	theVec = normalVec;

}
//����ʱ�������������ƶ���,�ü���
TopoDS_Edge SheetFlattenProcess::calRetractTranslate(SheetFlattenEdgeData & theEdge, const TopoDS_Face& theFace, const double theDistance, double theAngle) {
	// 1. ��ȡ�ߵ������˵�
	gp_Trsf translation;
	TopoDS_Edge a2dBaseEdge, a2dTarEdge, a2dNewEdge, a2dInterEdge;
	TopoDS_Edge newEdge;
	gp_Pnt p1, p2, intersection;
	bool isReduct = true;
	a2dBaseEdge = theEdge.getOldEdge_2d();
	theEdge.getNewEdge_2d(a2dNewEdge);
	TopoDS_Vertex w1 = TopExp::FirstVertex(a2dNewEdge);  // ��ȡ���
	TopoDS_Vertex w2 = TopExp::LastVertex(a2dNewEdge);   // ��ȡ�յ�

	p1 = BRep_Tool::Pnt(w1);  // ��ȡ�������
	p2 = BRep_Tool::Pnt(w2);

	TopoDS_Vertex v1 = TopExp::FirstVertex(a2dBaseEdge);  // ��ȡ���
	TopoDS_Vertex v2 = TopExp::LastVertex(a2dBaseEdge);   // ��ȡ�յ�

	gp_Pnt baseStart = BRep_Tool::Pnt(v1);  // ��ȡ�������
	gp_Pnt baseEnd = BRep_Tool::Pnt(v2);  // ��ȡ�յ�����


	gp_Vec normalVec;

	for (TopExp_Explorer explorer(theFace, TopAbs_EDGE); explorer.More(); explorer.Next()) {
		const TopoDS_Edge& currentEdge = TopoDS::Edge(explorer.Current());
		SheetFlattenEdgeData* pCurrentEdgedata;
		quiry3dEdgeData(currentEdge, pCurrentEdgedata);
		a2dTarEdge = pCurrentEdgedata->getOldEdge_2d();
		if (a2dBaseEdge == a2dTarEdge)
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
	if (fabs(M_PI - theAngle) <= 1e-6)
	{
		return newEdge;
	}
	if (theAngle >= M_PI / 2 && theAngle <= M_PI)
	{
		return newEdge;
	}
	vector<TopoDS_Edge> aInterEdges;
	if (theEdge.getVector_sameFace_interseEdge_2d(aInterEdges))
	{
		for (auto elem : aInterEdges)
		{
			SheetFlattenEdgeData* pEdgedata;
			quiryEdgeData(elem, pEdgedata);
			vector<TopoDS_Edge> aEdges;
			if (pEdgedata->getVector_newEdge_2d(aEdges))
			{
				if (pEdgedata->isBendEdge())
				{
					continue;
				}
				//if (!isReduct)
				{
					a2dInterEdge = aEdges.back();
					TrimEdgesAtIntersection1(newEdge, a2dInterEdge, intersection);
					//m_oldMapNewEdge[elem][aEdges.size()-1] = a2dInterEdge;
					pEdgedata->insertEdgeTo_new2d(a2dInterEdge);
				}
				//TrimEdgesAtIntersection1(newEdge, a2dInterEdge, intersection);
				//m_oldMapNewEdge[elem][0] = a2dInterEdge;
			}
		}
	}
	theEdge.insertEdgeTo_new2d(newEdge);
	return newEdge;  // �����µı�


}
//�������������ƶ���
void SheetFlattenProcess::calTranslate(const double distance, gp_Trsf& translation, const double& angle) {
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
TopoDS_Edge SheetFlattenProcess::midleLine(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2)
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
bool SheetFlattenProcess::TrimEdgesAtIntersection1(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection) {
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
bool SheetFlattenProcess::TrimEdgesAtIntersection(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection) {
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
bool SheetFlattenProcess::judgeless(double x, double y)
{
	double teee = y - x;
	if (y - x > 1e-6)
		return true;
	else
		return false;
}

//�ж������߶��ཻ
bool SheetFlattenProcess::EdgeIntersect(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2)
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
	if (judgeless(p1.X() > p2.X() ? p1.X() : p2.X(), p3.X() < p4.X() ? p3.X() : p4.X()) ||
		judgeless(p1.Y() > p2.Y() ? p1.Y() : p2.Y(), p3.Y() < p4.Y() ? p3.Y() : p4.Y()) ||
		judgeless(p3.X() > p4.X() ? p3.X() : p4.X(), p1.X() < p2.X() ? p1.X() : p2.X()) ||
		judgeless(p3.Y() > p4.Y() ? p3.Y() : p4.Y(), p1.Y() < p2.Y() ? p1.Y() : p2.Y()))
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
void SheetFlattenProcess::processSameFaceEdgeRelation()
{
	vector<TopoDS_Edge> intersectionLines, lines;
	for (auto &elem : m_EdgeData)
	{
		for (auto &item : elem)
		{
			auto faces = item.getVector_face();
			for (auto &it : faces)
			{


				for (TopExp_Explorer edgeExplorer(it, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
				{

					TopoDS_Edge otherEdge = TopoDS::Edge(edgeExplorer.Current());//��ά����
					if (otherEdge != item.getEdge_3d())
					{
						gp_Pnt intersection1;
						SheetFlattenEdgeData* pEdgedata;
						quiry3dEdgeData(otherEdge, pEdgedata);
						TopoDS_Edge twoEdge = pEdgedata->getOldEdge_2d();
						//TopoDS_Edge twoEdge = m_ThreeToTwoEdge.find(otherEdge)->second;//�ɶ�ά
						if (EdgeIntersect(twoEdge, item.getOldEdge_2d()))/*AreEdgesIntersecting(twoEdge, ThreeToTwoEdge.find(edge)->second)*/
						{
							intersectionLines.emplace_back(twoEdge);
						}
						else {
							lines.emplace_back(twoEdge);
						}
					}
				}

			}
			item.setVector_sameFace_interseEdge_2d(intersectionLines);
			item.setVector_sameFaceEdge_2d(lines);
			intersectionLines.clear();
			lines.clear();

		}
	}
}

//�ҵ�һ������Ľ���
bool SheetFlattenProcess::relationEdge(TopoDS_Edge edge)
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
void SheetFlattenProcess::allReation2dEdge()
{
	vector<TopoDS_Edge> interseEdges;
	for (auto &elem : m_EdgeData)
	{
		for (auto &base : elem)
		{
			TopoDS_Edge baseEdge = base.getOldEdge_2d();
			for (auto tar : elem)
			{
				TopoDS_Edge tarEdge = tar.getOldEdge_2d();
				if (baseEdge != tarEdge)
				{
					if (EdgeIntersect(baseEdge, tarEdge) && !isOverlap(baseEdge, tarEdge))
					{
						interseEdges.push_back(tarEdge);
						
					}
				}
			}
			base.setVector_interseEdge_2d(interseEdges);
			interseEdges.clear();
		}
	}
	//for (auto it : m_ThreeToTwoEdge)
	//{
	//	TopoDS_Edge baseEdge = it.second;
	//	for (auto elem : m_ThreeToTwoEdge)
	//	{
	//		TopoDS_Edge comEdge = elem.second;
	//		if (baseEdge != comEdge)
	//		{
	//			if (EdgeIntersect(baseEdge, comEdge) && !isOverlap(baseEdge, comEdge))
	//			{
	//				//interseEdges.push_back(comEdge);
	//			}
	//		}
	//	}
	//	//m_MapInterseLines[baseEdge] = interseEdges;
	//	//interseEdges.clear();
	//}
}
void SheetFlattenProcess::allReation2dNewEdge()
{
	TopoDS_Edge baseEdge, tarEdge;
	vector<TopoDS_Edge> interseEdges;
	for (auto &elem : m_EdgeData)
	{
		for (auto &base : elem)
		{
			vector<TopoDS_Edge> baseEdges;
			base.getVector_newEdge_2d(baseEdges);
			base.getNewEdge_2d(baseEdge);
			for (auto &tar : elem)
			{
				tar.getNewEdge_2d(tarEdge);
				vector<TopoDS_Edge> tarEdges;
				tar.getVector_newEdge_2d(tarEdges);
		
				if (baseEdge != tarEdge)
				{
					if (EdgeIntersect(baseEdge, tarEdge) && !isOverlap(baseEdge, tarEdge))
					{
						interseEdges.push_back(tar.getOldEdge_2d());
					}
				}
			}
			base.setVector_interseEdge_new2d(interseEdges);
			interseEdges.clear();
		}
	}



	//vector<TopoDS_Edge> interseEdges;
	//for (auto it : m_ThreeToTwoEdge)
	//{
	//	TopoDS_Edge baseEdge = it.second;
	//	findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, it.second, baseEdge);

	//	for (auto elem : m_ThreeToTwoEdge)
	//	{
	//		TopoDS_Edge comEdge = elem.second;
	//		findMap_EdgeAndVector_ToEdge(m_oldMapNewEdge, elem.second, comEdge);
	//		if (baseEdge != comEdge)
	//		{
	//			if (EdgeIntersect(baseEdge, comEdge) && !isOverlap(baseEdge, comEdge))
	//			{
	//				/*TopoDS_Edge old2dEdge;
	//				if(_2dTo3dEdge(m_newMapOldEdge, comEdge, old2dEdge))
	//				{
	//					interseEdges.push_back(old2dEdge);
	//				}
	//				else
	//				{
	//					interseEdges.push_back(comEdge);
	//				}*/
	//				interseEdges.push_back(elem.second);
	//			}
	//		}
	//	}
	//	m_MapInterseNewLines[it.second] = interseEdges;
	//	interseEdges.clear();
	//}
}

void SheetFlattenProcess::gerateNegativeBend(SheetFlattenEdgeData*& pEdgedata)
{
	gp_Pnt intersection;
	TopoDS_Edge newbaseEdge , newInterseEdge;
	vector<TopoDS_Edge> newBaseEdges;
	if (pEdgedata->isNegativeBend())
	{
		pEdgedata->getVector_newEdge_2d(newBaseEdges);

		TopoDS_Edge oldEdge, newEdge;
		oldEdge = newBaseEdges[1];
		newEdge = newBaseEdges[2];


		pEdgedata->getNewEdge_2d(newbaseEdge);
		vector<TopoDS_Edge> interEdges;
		pEdgedata->getVector_sameFace_interseEdge_2d(interEdges);
		//auto interEdges = pEdgedata->getVector_interseEdge_new2d();

		for (auto& itemm : interEdges)
		{
			SheetFlattenEdgeData* temptEdgedata = nullptr;
			quiryEdgeData(itemm, temptEdgedata);
			temptEdgedata->getNewEdge_2d(newInterseEdge);
			vector<TopoDS_Edge> aEdge, aVectorEdge1;

			if (!EdgeIntersect(newInterseEdge, newbaseEdge))
			{
				continue;
			}
			TrimEdgesAtIntersection1(newbaseEdge, newInterseEdge, intersection);//�ӳ��ߵ�����
			//oldMapNewEdge[edge] = newsideEdge;
			aEdge.emplace_back(newInterseEdge);
			if (temptEdgedata->getVector_newEdge_2d(aVectorEdge1))
			{
				if (aVectorEdge1.size() >= 3)
				{
					aEdge.clear();
					aEdge.emplace_back(aVectorEdge1[0]);
					aEdge.emplace_back(aVectorEdge1[1]);
					aEdge.emplace_back(newInterseEdge);
				}
			}
			temptEdgedata->setVector_newEdge_2d(aEdge);
			//m_oldMapNewEdge[itemm] = aEdge;
			aEdge.clear();
			aVectorEdge1.clear();
		}
	}
}
void SheetFlattenProcess::generateSoltEdge(const TopoDS_Edge& theEdge)
{
	gp_Pnt intersection;
	TopoDS_Edge newbaseEdge = theEdge, newInterseEdge;
	vector<TopoDS_Edge> baseEdges;
	SheetFlattenEdgeData* pEdgedata = nullptr;
	quiryEdgeData(theEdge, pEdgedata);
	pEdgedata->getVector_newEdge_2d(baseEdges);
	newbaseEdge = baseEdges[0];

	//pEdgedata->getNewEdge_2d(newbaseEdge);

	vector<TopoDS_Edge> interEdges;
	pEdgedata->getVector_sameFace_interseEdge_2d(interEdges);

	//auto interEdges = pEdgedata->getVector_interseEdge_new2d();
	
	for (auto &itemm : interEdges)
	{
		SheetFlattenEdgeData* temptEdgedata = nullptr;
		quiryEdgeData(itemm, temptEdgedata);
		temptEdgedata->getNewEdge_2d(newInterseEdge);
		vector<TopoDS_Edge> aEdge, aVectorEdge1;

		TrimEdgesAtIntersection1(newbaseEdge, newInterseEdge, intersection);//�ӳ��ߵ�����

		temptEdgedata->insertEdgeTo_new2d(newInterseEdge);
		//oldMapNewEdge[edge] = newsideEdge;
		aEdge.emplace_back(newInterseEdge);
		if (temptEdgedata->getVector_newEdge_2d(aVectorEdge1))
		{
			if (aVectorEdge1.size() >= 3)
			{
				aEdge.clear();
				aEdge.emplace_back(aVectorEdge1[0]);
				aEdge.emplace_back(aVectorEdge1[1]);
				aEdge.emplace_back(newInterseEdge);
			}
		}
		temptEdgedata->setVector_newEdge_2d(aEdge);
		//m_oldMapNewEdge[itemm] = aEdge;
		aEdge.clear();
		aVectorEdge1.clear();
	}
}

void SheetFlattenProcess::generateOverlapEdge(const TopoDS_Edge& theEdge)
{
	gp_Pnt intersection;
	SheetFlattenEdgeData* pEdgedata = nullptr;
	quiryEdgeData(theEdge, pEdgedata);
	TopoDS_Edge newbaseEdge, newInterseEdge;
	vector<TopoDS_Edge> IneterEdges;
	pEdgedata->getNewEdge_2d(newbaseEdge);
	if (pEdgedata->getVector_interseEdge_new2d(IneterEdges))
	{
		for (auto &itemm : IneterEdges)
		{
			vector<TopoDS_Edge> aEdge, aVectorEdge1;
			SheetFlattenEdgeData* pTarEdgedata = nullptr;
			quiryEdgeData(itemm, pTarEdgedata);
			if (!pTarEdgedata->isOverlapeEdge())
			{
				continue;
			}
			pTarEdgedata->getNewEdge_2d(newInterseEdge);

			TrimEdgesAtIntersection1(newbaseEdge, newInterseEdge, intersection);//�ӳ��ߵ�����
			//oldMapNewEdge[edge] = newsideEdge;
			aEdge.emplace_back(newInterseEdge);
			if (pTarEdgedata->getVector_newEdge_2d(aVectorEdge1))
			{
				if (aVectorEdge1.size() >= 3)
				{
					aEdge.clear();
					aEdge.emplace_back(aVectorEdge1[0]);
					aEdge.emplace_back(aVectorEdge1[1]);
					aEdge.emplace_back(newInterseEdge);
				}
			}
			pTarEdgedata->setVector_newEdge_2d(aEdge);
			//pTarEdgedata->setOverLapEdge_Point(intersection);
			aEdge.clear();
		}
	}
	
}
void SheetFlattenProcess::generateEdge(const TopoDS_Edge& theEdge)
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
bool SheetFlattenProcess::CalculateAngleBetweenFaces(const TopoDS_Face& theBaseFace, const TopoDS_Face& theTarFace, double& angle) {

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
bool SheetFlattenProcess::CalculateAngleBetweenFaces(const vector<TopoDS_Face>& face, double& angle) {

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
void SheetFlattenProcess::wrapAngle()
{
	double angle;
	double distance = 2.;
	for (auto &elem : m_EdgeData)
	{
		for (auto &item : elem)
		{
			for (auto& it : elem)
			{
				if (item.isProcessWrapAngle() || it.isProcessWrapAngle())
				{
					continue;
				}
				if (it.getEdge_3d() != item.getEdge_3d())
				{
					if (AreEdgesSameEndpoints(it.getEdge_3d(), item.getEdge_3d()))
					{
						vector<TopoDS_Face> aBaseFaces, aBaseTarFaces;
						aBaseFaces = item.getVector_face();
						aBaseTarFaces = it.getVector_face();
						if (aBaseFaces.size() < 2 && aBaseTarFaces.size() < 2)
						{
							CalculateAngleBetweenFaces(aBaseFaces[0], aBaseTarFaces[0], angle);
							item.setAngle(angle);
							item.setProcessWrapAngle(true);
							it.setProcessWrapAngle(true);
							it.setWrapAngle(true);
							it.setAngle(angle);
							//calRetractTranslate(it, aBaseTarFaces[0], distance, angle);
						}
					}
				}
			}
			if (item.isWrapAngle())
			{
				angle = item.getAngle();
				TopoDS_Face face = item.getVector_face()[0];

				calRetractTranslate(item, face, distance, angle);
			}

		}
	}
}
void SheetFlattenProcess::processOutline()
{
	for (auto &elem : m_EdgeData)
	{
		for(auto &item:elem)
		{
			if (!item.isBendEdge())
			{
				TopoDS_Edge aBaseEdge;
				if (item.isOutline())
				{
					aBaseEdge = item.getOutlineEdge();
				}
				else {
					item.getNewEdge_2d(aBaseEdge);
				}
				vector<TopoDS_Edge> aTarVectorEdges;
				TopoDS_Edge aTarEdge;
				if (item.getVector_interseEdge_new2d(aTarVectorEdges))
				{
					for (auto &tarElem : aTarVectorEdges)
					{
						SheetFlattenEdgeData* pTarEdgadata = nullptr;
						quiryEdgeData(tarElem, pTarEdgadata);
						vector<TopoDS_Edge> aTarVectorNewEdges;
						if (pTarEdgadata->getVector_newEdge_2d(aTarVectorNewEdges))
						{
							if (aTarVectorNewEdges.size() < 3)
							{
								if (pTarEdgadata->isOutline())
								{
									aTarEdge = pTarEdgadata->getOutlineEdge();
								}
								else {
									pTarEdgadata->getNewEdge_2d(aTarEdge);
								}
								gp_Pnt intersection;
								TrimEdgesAtIntersection1(aBaseEdge, aTarEdge, intersection);
								item.setOutline(true);
								pTarEdgadata->setOutline(true);
								item.setOutlineEdge(aBaseEdge);
								pTarEdgadata->setOutlineEdge(aTarEdge);
								//aTarVectorEdges[aTarVectorEdges.size() - 1] = aBaseEdge;
								//aTarVectorNewEdges[aTarVectorNewEdges.size() - 1] = aTarEdge;
								//item.setVector_newEdge_2d(aTarVectorEdges);
								//pTarEdgadata->setVector_newEdge_2d(aTarVectorNewEdges);

							}
						}
					}
				}
			}
		}
	}
}


void SheetFlattenProcess::check()
{
	TopoDS_Compound aCompound;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aCompound);
	for (auto elem : m_EdgeData)
	{
		for (auto item : elem)
		{
			TopoDS_Edge edge;
			vector<TopoDS_Edge> aEdges;
			if (item.isOutline())
			{
				edge = item.getOutlineEdge();
				aBuilder.Add(aCompound, edge);
			}
			if (item.getVector_newEdge_2d(aEdges))
			{
				for (auto it : aEdges)
				{
					//aBuilder.Add(aCompound, it);
				}
			}
			else
			{
				edge = aEdges[0];
				aBuilder.Add(aCompound, edge);
			}
		}
	}

	for (auto& elem : m_outlindeEdges)
	{
		aBuilder.Add(aCompound, elem);
	}
	FlattenFace::BuildDocStd(aCompound);
}


double CalculateEdgeDistance(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2) {
	// ��ȡ�����ߵļ�������
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// ��ȡ�ڶ����ߵ������˵�
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt p2_1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt p2_2 = BRep_Tool::Pnt(v2_2);

	gp_Vec base(p1_1, p1_2);
	gp_Vec tar(p1_1, p2_1);
	gp_Vec tar1(p1_1, p2_2);


	gp_Vec crossProduct = tar.Crossed(base);
	double distance = crossProduct.Magnitude() / base.Magnitude();

	gp_Vec crossProduct1 = tar1.Crossed(base);
	double distance1 = crossProduct1.Magnitude() / base.Magnitude();

	if (distance > distance1)
	{
		return distance1;
	}
	return distance;
}

// �����������ڵ�λ�����жϷ���һ��
bool AreVectorsSameDirectionNormalized(const gp_Vec& vec1, const gp_Vec& vec2) {
	gp_Vec normVec1 = vec1.Normalized();
	gp_Vec normVec2 = vec2.Normalized();
	double diff = (normVec1 - normVec2).Magnitude();
	return diff < 1e-6; // ʹ��С���ֵ�ж����
}

void SheetFlattenProcess::moveSplitEdge()
{
	if (m_adjGroupIndex.size() == 0)return;
	gp_Vec standardVec = m_adjGroupIndex.begin()->second[0].moveVec;
	double distance=0.;
	double otherdistance = 0.;
	vector<double> allDistance;
	vector<double> otherDistance;
	for (auto elem : m_adjGroupIndex)
	{
		for (auto item : elem.second)
		{
			if (AreVectorsSameDirectionNormalized(standardVec, item.moveVec))
			{
				distance += item.distance;
			}
			else {
				otherdistance += item.distance;
			}
			for (auto& it : m_EdgeData[item.adjEdgeDataIndex])
			{
				gp_Vec translationVec = item.moveVec * distance;
				gp_Trsf translation;
				translation.SetTranslation(translationVec);

				TopoDS_Edge newedge = TranslateEdge(it.getOldEdge_2d(), translation);
				changeMapIdex(it.getOldEdge_2d(), newedge);
				it.setOldEdge_2d(newedge);
			}
		}
		check();
	}
}

// ���������߶ε�˳ʱ��н�
double CalculateEdgeClockwiseAngle(const TopoDS_Edge & edge1, const TopoDS_Edge & edge2,const gp_Dir& referenceAxis) {
	// �������߶εķ�������

	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p2 = BRep_Tool::Pnt(v1_2);

	// ��ȡ�ڶ����ߵ������˵�
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt q1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt q2 = BRep_Tool::Pnt(v2_2);

	gp_Vec vec1(p1, p2);
	gp_Vec vec2(q1, q2);

	// �����׼�н�
	double dotProduct = vec1 * vec2;
	double magnitudeProduct = vec1.Magnitude() * vec2.Magnitude();
	double angle = acos(dotProduct / magnitudeProduct); // ����ֵ�� [0, ��]

	// ʹ�ò���жϷ���
	gp_Vec crossProduct = vec1.Crossed(vec2); // vec1 �� vec2
	double direction = crossProduct * gp_Vec(referenceAxis); // ����жϷ���

	// �������Ϊ������н���˳ʱ�룬��������ʱ��
	if (direction < 0) {
		angle = 2 * M_PI - angle; // תΪ˳ʱ�뷽��
	}

	return angle; // ����ֵ��Χ [0, 2��]
}



// ������������֮���˳ʱ��нǣ�����Ϊ�ο���
double CalculateClockwiseAngle(const gp_Vec& vec1, const gp_Vec& vec2, const gp_Dir& referenceAxis) {
	// �����׼�н�
	double dotProduct = vec1 * vec2;
	double magnitudeProduct = vec1.Magnitude() * vec2.Magnitude();
	double angle = acos(dotProduct / magnitudeProduct); // ����ֵ�� [0, ��]

	double baseAngle = angle;
	// ʹ�ò���жϷ���
	gp_Vec crossProduct = vec1.Crossed(vec2); // vec1 �� vec2
	double direction = crossProduct * gp_Vec(referenceAxis); // ����жϷ���

	// �������Ϊ������н���˳ʱ�룬��������ʱ��
	if (direction < 0) {
		angle = 2 * M_PI - angle; // תΪ˳ʱ�뷽��
	}
	if (angle == M_PI)
	{
		return 0.;
	}
	else if (fabs(angle - 0.) < 1e-6)
	{
		return M_PI;
	}
	else if (angle > M_PI)
	{
		angle = M_PI + baseAngle;
	}
	else if (angle < M_PI)
	{
		angle = M_PI - baseAngle;
	}

	return angle; // ����ֵ��Χ [0, 2��]
}

// ��һ������ԭ��Ϊ������תһ���Ƕ�
TopoDS_Edge RotateEdge(const TopoDS_Edge& edge, double angleInDegrees) {
	// ���ǶȴӶ�ת��Ϊ����
	double angleInRadians = angleInDegrees;// * M_PI / 180.0;

	// ������ת�ᣨ��ԭ��Ϊ���ģ��� Z ����ת��
	gp_Ax1 rotationAxis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));

	// ������ת�任
	gp_Trsf rotationTransform;
	rotationTransform.SetRotation(rotationAxis, angleInRadians);

	// Ӧ�ñ任����
	BRepBuilderAPI_Transform transformer(edge, rotationTransform);
	return TopoDS::Edge(transformer.Shape());
}

void SheetFlattenProcess::processSplitEdge()
{
	double maxDistance = -1;
	gp_Dir standard(0, 0, 1);
	gp_Vec moveVec,baseVec,tarVec;
	bool isAdj = false;
	int i = -1, j = -1;
	int idexj = -1;
	int baseIndex = -1, tarIndex = -1, maxBaseIndex = -1, maxTarIndex = -1;
	for (auto& baseGroup : m_EdgeData) {
		++i;
		for (auto& targetGroup : m_EdgeData) {
			++j;
			// �����ͬһ�����飬����
			if (&baseGroup == &targetGroup) continue;

			// ����������
			for (auto& base : baseGroup) {
				++baseIndex;
				// ����������Ѵ����������
				if (base.isProcessWrapAngle()) continue;

				for (auto& target : targetGroup) {
					++tarIndex;
					// ���Ŀ����Ѵ���������߶˵㲻ͬ������
					if (target.isProcessWrapAngle() ||
						!AreEdgesSameEndpoints(base.getEdge_3d(), target.getEdge_3d())) {
						continue;
					}
					

					double distance = CalculateEdgeDistance(base.getOldEdge_2d(), target.getOldEdge_2d());
					if (distance > maxDistance)
					{
						idexj = j;
						isAdj = true;
						maxDistance = distance;
						maxBaseIndex = baseIndex;
						maxTarIndex = tarIndex;
					}


					// ��ȡ�����ߺ�Ŀ��ߵ�������
					const auto& baseFaces = base.getVector_face();
					const auto& targetFaces = target.getVector_face();

					// �������ߵĹ��������� >= 2������
					if (baseFaces.size() >= 2 || targetFaces.size() >= 2) continue;

					// �������ߵļн�
					double angle1 = 0.0;
					CalculateAngleBetweenFaces(baseFaces[0], targetFaces[0], angle1);

					// ���»����ߺ�Ŀ��ߵ�����
					base.setAngle(angle1);
					base.setProcessWrapAngle(true);

					target.setProcessWrapAngle(true);
					target.setWrapAngle(true);
					target.setAngle(angle1);
				}
				
				tarIndex = -1;
			}
			if (maxTarIndex != -1)
			{

				calTranslate(baseGroup[maxBaseIndex], baseGroup[maxBaseIndex].getVector_face()[0], baseVec);
				calTranslate(targetGroup[maxTarIndex], targetGroup[maxTarIndex].getVector_face()[0], tarVec);
				double angle = CalculateClockwiseAngle(baseVec, tarVec, standard);

				double teset = fabs(angle - M_PI);
				bool isrotate = false;
				//angle = CalculateEdgeClockwiseAngle(base.getNewEdge_2d(), target.getNewEdge_2d(), standard);
				if (fabs(angle) > 1e-2 && fabs(angle - M_PI) > 1e-2)
				{
					for (auto& elem : targetGroup)
					{
						TopoDS_Edge newEdge = RotateEdge(elem.getOldEdge_2d(), angle);
						isrotate = true;
						changeMapIdex(elem.getOldEdge_2d(), newEdge);
						elem.setOldEdge_2d(newEdge);
						//elem.insertEdgeTo_new2d(newEdge);
					}
				}
				check();


				if (isrotate)
				{
					maxDistance = -1;
					tarIndex = -1;
					baseIndex = -1;
					for (auto& base : baseGroup) {
						++baseIndex;
						// ����������Ѵ����������
						if (base.isProcessWrapAngle()) //continue;

						for (auto& target : targetGroup) {
							++tarIndex;
							// ���Ŀ����Ѵ���������߶˵㲻ͬ������
							if (!AreEdgesSameEndpoints(base.getEdge_3d(), target.getEdge_3d())) {
								continue;
							}


							double distance = CalculateEdgeDistance(base.getOldEdge_2d(), target.getOldEdge_2d());
							if (distance > maxDistance)
							{
								idexj = j;
								isAdj = true;
								maxDistance = distance;
								maxBaseIndex = baseIndex;
								maxTarIndex = tarIndex;
							}

						}
						tarIndex = -1;
					}
					baseIndex = -1;
				}

				calTranslate(targetGroup[maxTarIndex], targetGroup[maxTarIndex].getVector_face()[0], moveVec);
				//maxDistance = CalculateEdgeDistance(baseGroup[maxBaseIndex].getOldEdge_2d(), targetGroup[maxTarIndex].getOldEdge_2d());
			}
			maxTarIndex = -1;
			maxBaseIndex = -1;
			baseIndex = -1;
		}
		if (isAdj == true)
		{
			//if (m_adjGroupIndex.find(i) == m_adjGroupIndex.end())
			{
				moveData data;
				data.adjEdgeDataIndex = idexj;
				data.distance = maxDistance;
				data.moveVec = moveVec;
				m_adjGroupIndex[i].emplace_back(data);
			}

		}
		isAdj = false;
		idexj = -1;
		j = -1;
		maxDistance = -1.;
	}
}

void SheetFlattenProcess::processEdges_WrapAngle() {
	// ����ÿ������
	for (auto& baseGroup : m_EdgeData)
	{
		// �ƶ���δ��ɵı�
		for (auto& edge : baseGroup) 
		{
			if (!edge.isFinish()) 
			{
				moveEdge(baseGroup, edge);
				check();
			}
		}
	}
}



void SheetFlattenProcess::generate(TopoDS_Compound& aCompound)
{
	check();
	processSplitEdge();
	check();
	moveSplitEdge();
	check();

	allReation2dEdge();
	processSameFaceEdgeRelation();
	processOverlap();
	allReation2dNewEdge();


	processEdges_WrapAngle();
	check();
	
	makeLapelMap();//�۱�
	translateLapel(1.);
	check();
	wrapAngle();
	set<TopoDS_Edge> FinishEdges;
	check();

	for (auto &elem : m_EdgeData)
	{
		for (auto &item : elem)
		{

			if (FinishEdges.find(item.getOldEdge_2d()) != FinishEdges.end())
			{
				continue;
			}
			TopoDS_Edge aBaseEdge = item.getOldEdge_2d();
			for (auto &it : elem)
			{
				TopoDS_Edge aTarEdge = it.getOldEdge_2d();
				if (aBaseEdge != aTarEdge)
				{
					if (EdgeIntersect(aBaseEdge, aTarEdge))
					{
						if (it.isOverlapeEdge() && item.isOverlapeEdge())
						{
							vector<TopoDS_Edge> aVectorEdges;
							gp_Pnt aBasePoint = it.getOverLapEdge_Point();
							gp_Pnt aTarPoint = item.getOverLapEdge_Point();
							TopoDS_Edge aNewEdge = BRepBuilderAPI_MakeEdge(aBasePoint, aTarPoint);
							if (it.getVector_newEdge_2d(aVectorEdges))
							{
								FinishEdges.insert(aBaseEdge);
								//it.insertEdgeTo_new2d(aNewEdge);
								m_outlindeEdges.emplace_back(aNewEdge);
							}

						}
					}

				}
			}
		}
	}
	check();

	processOutline();
	/*makeLapelMap();
	translateLapel(1.);
	wrapAngle();*/
	check();
}
void SheetFlattenProcess::generateNewEdge(const vector<TopoDS_Edge>& theMoveEdge, const gp_Trsf& translation, bool theMove)
{
	for (auto it : theMoveEdge)
	{
		SheetFlattenEdgeData* aEdgadata = nullptr;
		quiryEdgeData(it, aEdgadata);
		vector<TopoDS_Edge> aEdges;
		vector<TopoDS_Edge> tempedge;
		aEdgadata->getVector_newEdge_2d(tempedge);
		if (theMove)
		{
			if (aEdgadata->isOverlapeEdge())
			{
				gp_Pnt aBasePoint = aEdgadata->getOverLapEdge_Point();
				aBasePoint.Transform(translation);
				aEdgadata->setOverLapEdge_Point(aBasePoint);
			}
			for (auto elem : tempedge)
			{
				TopoDS_Edge newedge = TranslateEdge(elem, translation);
				aEdges.emplace_back(newedge);
			}
			aEdgadata->setVector_newEdge_2d(aEdges);
			aEdges.clear();
		}
		else
		{
			aEdgadata->setVector_newEdge_2d(tempedge);
		}

	}
}
void SheetFlattenProcess::generateMidleEdge(const vector<TopoDS_Edge>& theMoveEdge, const gp_Trsf& theTranslation, const gp_Trsf& theTranslation_slot)
{

	vector<TopoDS_Edge> aEdges;
	for (auto it : theMoveEdge)//�������ߵ���
	{
		SheetFlattenEdgeData* aEdgadata;
		quiryEdgeData(it, aEdgadata);
		if (aEdgadata->getVector_face().size() >= 2)
		{
			if (m_isNegativeBend)
			{
				aEdgadata->setNegativeBend(true);
			}
			aEdgadata->setFinish(true);
			aEdgadata->setBendEdge(true);
			TopoDS_Edge tempedge;
			aEdgadata->getNewEdge_2d(tempedge);

			TopoDS_Edge newedge, aSoltEdge1, aSoltEdge2;
			if (aEdgadata->isSoltEdge())
			{
				newedge = TranslateEdge(tempedge, theTranslation_slot);
			}
			else {
				newedge = TranslateEdge(tempedge, theTranslation);
			}
			TopoDS_Edge midedge = midleLine(tempedge, newedge);
			aEdges.emplace_back(midedge);
			aEdges.emplace_back(newedge);
			aEdges.emplace_back(tempedge);
			aEdgadata->setVector_newEdge_2d(aEdges);
			aEdges.clear();

		}
		else {

			aEdgadata->setFinish(true);
			TopoDS_Edge tempedge;
			aEdgadata->getNewEdge_2d(tempedge);
			//TopoDS_Edge newedge = TranslateEdge(tempedge, theTranslation);
			//TopoDS_Edge midedge = midleLine(tempedge, newedge);
			
			aEdges.emplace_back(tempedge);
			aEdgadata->setVector_newEdge_2d(aEdges);
			aEdges.clear();
		}
	}
}
void SheetFlattenProcess::moveEdge(vector<SheetFlattenEdgeData> &edges,SheetFlattenEdgeData &edge)
{
	BRep_Builder aBuilder;
	vector<TopoDS_Edge> leftEdges;
	vector<TopoDS_Edge> rightEdges;
	vector<TopoDS_Edge> colines;
	vector<TopoDS_Edge> otherlines;
	vector<TopoDS_Face> faces;
	if (edge.getVector_face().size()>=2)
	{

		edge.setFinish(true);
		gp_Trsf translation, translation_slot;
		Standard_Real angle = edge.getAngle();
		//CalculateAngleBetweenFaces(faces, angle);

		TopoDS_Edge baseEdge = edge.getOldEdge_2d();//�ɶ�ά
		FindEdgesOnBothSides(edges, edge, leftEdges, rightEdges, colines, otherlines);//��������װ�Ķ��Ǿɶ�ά,����Ų
		/*if (angle > M_PI)
		{
			m_baseVec = -m_baseVec;
		}*/
		double moveDis = 2.;
		calTranslate(moveDis, translation_slot, angle);
		calTranslate(moveDis, translation, angle);//ȷ���ƶ�����ͽǶ�
		if (angle > M_PI)
		{
			m_isNegativeBend = false;
		}
		else
		{
			m_isNegativeBend = true;
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
			generateNewEdge(leftEdges, translation, true);
			generateNewEdge(otherlines, translation, true);
		}
	


		for (auto &it : colines)//�������ߵ���
		{
			if (m_isNegativeBend)
			{
				//generateEdge(it);
				//generateSoltEdge(it);
			}
			//generateOverlapEdge(it);
			//generateEdge(it);
		}


	}
}
bool SheetFlattenProcess::getAdjFace(TopoDS_Edge edge, vector<TopoDS_Face>& result)
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
bool SheetFlattenProcess::AreEdgesSameEndpoints(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2) {
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
void SheetFlattenProcess::findSameEdge(map<TopoDS_Edge, TopoDS_Edge>& result)
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
			if (AreEdgesSameEndpoints(aBaseEdge, aTarEdge))
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