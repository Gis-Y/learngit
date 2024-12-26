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
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_Circle.hxx>
#include <gp_Circ.hxx>
#include<GeomAPI_IntCS.hxx>


#include <Geom_Line.hxx>
#include <Geom_Circle.hxx>
#include <Geom_TrimmedCurve.hxx>

#include <BRep_Tool.hxx>
#include <Geom_Curve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <gp_Pnt.hxx>
#include <stdexcept>


#include "SheetFlattenEdgeData.h"
#include"UniversalOtherSizeCalculate.h"

#include <algorithm>
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
	TopoDS_Edge translateEdge(const TopoDS_Edge& edge, const gp_Trsf& translation);
	void processEdges_WrapAngle();
	void processSplitEdge();
	void QuirySplitEdge();
	void processSameFaceEdgeRelation();
	bool isOverlap(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2);
	gp_Pnt findOtherPoint(const TopoDS_Edge& theEdge, const gp_Pnt& thePoint);
	void allReation2dEdge();
	TopoDS_Edge midleLine(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2);
	void FindEdgesOnBothSides(vector<SheetFlattenEdgeData>& edges, SheetFlattenEdgeData& baseEdge,
		std::vector<TopoDS_Edge>& leftEdges, std::vector<TopoDS_Edge>& rightEdges, std::vector<TopoDS_Edge>& colines, std::vector<TopoDS_Edge>& otherlines);
	void calTranslate(const double distance, gp_Trsf& translation, const double& angle);//无用可以注释
	TopoDS_Edge calRetractTranslate(SheetFlattenEdgeData &theEdge, const TopoDS_Face& theFace, const double theDistance, double theAngle);
	bool CalculateAngleBetweenFaces(const vector<TopoDS_Face>& face, double& angle);
	bool CalculateAngleBetweenFaces(const TopoDS_Face& theBaseFace, const TopoDS_Face& theTarFace, double& angle);
	void moveEdge(vector<SheetFlattenEdgeData>& edges, SheetFlattenEdgeData& edge);
	void processOverlap();
	TopoDS_Edge findAdjEdge(const TopoDS_Edge &baseEdge, const vector<TopoDS_Edge> &interEdges, const gp_Pnt &basePoint);
	void generateMidleEdge(const vector<TopoDS_Edge>& theMoveEdge, const gp_Trsf& translation, const gp_Trsf& theTranslation_slot);
	void generateNewEdge(const vector<TopoDS_Edge>& theMoveEdge, const gp_Trsf& translation, bool theMove);
	void generateSoltEdge(const TopoDS_Edge& theEdge);
	void gerateNegativeBend(SheetFlattenEdgeData*& pEdgedata);
	void generateOverlapEdge(const TopoDS_Edge& theEdge);
	bool SegmentsOverlap(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, TopoDS_Edge& newEdge1, TopoDS_Edge& newEdge2,
		gp_Pnt& newPoint1, gp_Pnt& newPoint2, gp_Pnt& startPoint1, gp_Pnt& startPoint2, gp_Vec& directionVec1, gp_Vec& directionVec2);
	void processFlotInVec(gp_Vec& normalVec);
	gp_Vec CalMoveVector(const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p);
	bool calTranslate(SheetFlattenEdgeData& theEdge, const TopoDS_Face& theFace, gp_Vec& theVec);
	int GetPointSide1(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point);
	int GetPointSide(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point);
	void allReation2dNewEdge();
	bool judgeless(double x, double y);//无用可以注释
	bool EdgeIntersect(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2);//无用可以注释
	bool TrimEdgesAtIntersection1(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection);//无用可以注释
	bool TrimEdgesAtIntersection(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection);//无用可以注释
	TopoDS_Edge TranslatePositiveEdge(const TopoDS_Edge& edge, const gp_Trsf& translation, TopoDS_Edge& theNewEdge, TopoDS_Edge& theNewEdge2);
	gp_Pnt GetPerpendicularFootPoint(const gp_Pnt& point, const TopoDS_Edge& edge);
	bool quiryNewEdge_2d(TopoDS_Edge& baseEdge, TopoDS_Edge& tarEdge);
	bool quiryNewEdge_2d(TopoDS_Edge& baseEdge, vector<TopoDS_Edge>& tarEdge);
	bool quiryEdgeData(const TopoDS_Edge& baseEdge, SheetFlattenEdgeData*& data);
	void changeMapIdex(const TopoDS_Edge& baseEdge, const TopoDS_Edge& tarEdge);
	bool quiry3dEdgeData(const TopoDS_Edge& baseEdge, SheetFlattenEdgeData*& data);
	void wrapAngle();
	void makeLapelMap();
	void processOutline();
	void translateLapel(const double theDistance);
	bool isSameFace(const vector<TopoDS_Face>& theLeftFaces, const vector<TopoDS_Face>& theRightFaces);
	bool AreEdgesSameEndpoints(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2);
	bool IsEdgesAtIntersection(TopoDS_Edge& edge1, TopoDS_Edge& edge2, gp_Pnt& intersection);
	void calTranslateOriention(const TopoDS_Edge& theEdge, const gp_Pnt& thePoint, gp_Trsf& theTranslation, const double theDistance);
	void ToInfo_DXF(const TopoDS_Edge& edge, point_t& startPoint, point_t& endPoint, point_t& center, Standard_Real& radius,
		Standard_Real& startAngle, Standard_Real& endAngle, int& edgeType);
public:
	void check();
	void generate();
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

	gp_Dir m_refDir;
	gp_Vec m_baseVec;//移动方向  放
	bool m_isNegativeBend;
	vector<TopoDS_Edge> m_outlindeEdges;
	std::vector<std::tuple<point_t, point_t, std::string>> m_line;
	std::vector<std::tuple<point_t, double, std::string>> m_circle;
	std::vector< std::tuple< point_t, double, double, double, std::string > > m_eclipse;
};

SheetFlattenProcess::SheetFlattenProcess()
{
}

SheetFlattenProcess::~SheetFlattenProcess()
{
}

void SheetFlattenProcess::changeMapIdex(const TopoDS_Edge& oldEdge, const TopoDS_Edge& newEdge)
{
	auto it = m_oldEdgeIndex.find(oldEdge); // 找到键为 2 的元素
	if (it != m_oldEdgeIndex.end()) {
		m_oldEdgeIndex[newEdge] = it->second; // 插入新键值对，值保持不变
		m_oldEdgeIndex.erase(it);       // 删除旧键值对
	}
}

double BendingSizeCalculate(const double angle, const double SheetThickness)
{
	double Result = 0;
	if (angle > 180) 
	{
		std::map<double, double> NegativeBendingChangerules = {
			{0.5, 0}, {0.6, 0.3}, {0.8, 0.5}, {1, 0.8}, {1.2, 1}, {1.5, 1}, {2, 1.5},
			{2.5, 2.0}, {3, 2.5}, {4, 3.0}, {5, 4.0}, {6, 5.0}, {8, 6.5}, {10, 8.0}, {12, 9.5}
		};
		double ThisCaseBendingSize_90 = 0;
		auto it = NegativeBendingChangerules.find(SheetThickness);
		if (it != NegativeBendingChangerules.end())
		{
			ThisCaseBendingSize_90 = it->second;
		}
		else
		{
			ThisCaseBendingSize_90 = 0;
		}

		double K1Temp = class_OtherSizeCalculate.PositiveK1(ThisCaseBendingSize_90, angle);
		Result = class_OtherSizeCalculate.NnegativeK2(SheetThickness, angle, K1Temp);
	}
	else if (angle <= 180) 
	{
		
		std::map<double, double> PositiveBendingChangerules = {
			{0.5, 0}, {0.6, 0.3}, {0.8, 0.5}, {1, 0.8}, {1.2, 1}, {1.5, 1}, {2, 1.5},
			{2.5, 2.0}, {3, 2.5}, {4, 3.0}, {5, 4.0}, {6, 5.0}, {8, 6.5}, {10, 8.0}, {12, 9.5}
		};
		double ThisCaseBendingSize_90 = 0;

		auto it = PositiveBendingChangerules.find(SheetThickness);
		if (it != PositiveBendingChangerules.end())
		{
			ThisCaseBendingSize_90 = it->second;
		}
		else
		{
			ThisCaseBendingSize_90 = 0;
		}

		Result = class_OtherSizeCalculate.PositiveK1(ThisCaseBendingSize_90, angle);
	}

	return Result;
}


double SolttingSizeCalculate(const double angle, const double sheetThinkness)
{
	double Result = 0;
	if (angle > 180) 
	{
		Result = 0;
	}
	else if (angle <= 180) 
	{
	
		if (sheetThinkness < 1.2)
		{
	
			std::map<double, double> PositiveBendingChangerules = {
				{0.5, 0}, {0.6, 0.3}, {0.8, 0.5}, {1, 0.8}, {1.2, 1}, {1.5, 1}, {2, 1.5},
				{2.5, 2.0}, {3, 2.5}, {4, 3.0}, {5, 4.0}, {6, 5.0}, {8, 6.5}, {10, 8.0}, {12, 9.5}
			};
			double ThisCaseBendingSize_90 = 0;

			auto it = PositiveBendingChangerules.find(sheetThinkness);
			if (it != PositiveBendingChangerules.end())
			{
				ThisCaseBendingSize_90 = it->second;
			}
			else
			{
				ThisCaseBendingSize_90 = 0;
			}

			Result = std::abs(std::round(ThisCaseBendingSize_90 * (1 / std::tan(angle / 2 * M_PI / 180)) * 10) / 10);
		}
		else
		{
			Result = std::abs(std::round(1 * (1 / std::tan(angle / 2 * M_PI / 180)) * 10) / 10);
		}
	}
	return Result;
}
// 判断两条线段是否近乎平行
bool areEdgesNearlyParallel(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, double angleThresholdDegrees) {
	// 提取边的端点
	Standard_Real f1, l1, f2, l2;
	Handle(Geom_Curve) curve1 = BRep_Tool::Curve(edge1, f1, l1);
	Handle(Geom_Curve) curve2 = BRep_Tool::Curve(edge2, f2, l2);

	if (curve1.IsNull() || curve2.IsNull()) {
		std::cerr << "无法提取曲线，检查输入的边是否有效。" << std::endl;
		return false;
	}

	// 获取两条边的起点和终点
	gp_Pnt p1Start, p1End, p2Start, p2End;
	curve1->D0(f1, p1Start);
	curve1->D0(l1, p1End);
	curve2->D0(f2, p2Start);
	curve2->D0(l2, p2End);

	// 计算方向向量
	gp_Vec vec1(p1Start, p1End);
	gp_Vec vec2(p2Start, p2End);

	// 检查向量长度是否有效
	if (vec1.Magnitude() < 1e-7 || vec2.Magnitude() < 1e-7) {
		std::cerr << "方向向量长度过小，无法计算平行性。" << std::endl;
		return false;
	}

	// 计算点积和余弦值
	Standard_Real dotProduct = vec1.Dot(vec2);
	Standard_Real cosTheta = dotProduct / (vec1.Magnitude() * vec2.Magnitude());

	// 余弦的绝对值接近1表示近乎平行（夹角接近0或180度）
	double cosThreshold = std::cos(angleThresholdDegrees * (M_PI / 180.0));

	bool data = std::abs(cosTheta) >= cosThreshold;
	return std::abs(cosTheta) >= cosThreshold;
}

bool FindIntersectionAndExtend(TopoDS_Edge& edge1,TopoDS_Edge& edge2,gp_Pnt& intersectionPoint) 
{
	// 创建求交对象
	Standard_Real baseFirst, baseLast, tarFirst, tarLast;
	Handle(Geom_Curve) curve1 = BRep_Tool::Curve(edge1, baseFirst, baseLast);
	Handle(Geom_Curve) curve2 = BRep_Tool::Curve(edge2, tarFirst, tarLast);

	GeomAPI_ExtremaCurveCurve extrema(curve1, curve2);

	if (extrema.NbExtrema() == 0) {
		std::cerr << "No intersection found!" << std::endl;
		return false;
	}

	// 遍历找到最近点对
	for (int i = 1; i <= extrema.NbExtrema(); ++i) {


		try {
			Standard_Real distance = extrema.Distance(i);

			// 检查距离是否在容差范围内
			if (distance < Precision::Confusion()) {
				gp_Pnt interPoint1, interPoint2;
				extrema.Points(i, interPoint1, interPoint2);

				// 记录交点
				intersectionPoint = interPoint1;
				Standard_Real param1, param2;
				extrema.Parameters(i, param1, param2);

				if (fabs(param1 - baseFirst) > fabs(param1 - baseLast))
				{
					edge1 = BRepBuilderAPI_MakeEdge(curve1, baseFirst, param1).Edge();
				}
				else
				{
					edge1 = BRepBuilderAPI_MakeEdge(curve1, baseLast, param1).Edge();
				}


				// 延长直线

				if (fabs(param2 - tarFirst) > fabs(param2 - tarLast))
				{
					edge2 = BRepBuilderAPI_MakeEdge(curve2, tarFirst, param2).Edge();
				}
				else
				{
					edge2 = BRepBuilderAPI_MakeEdge(curve2, tarLast, param2).Edge();
				}

				return true;
			}
		}
		catch (Standard_Failure& err)
		{
			std::cerr << "Error while checking intersection: " << err.GetMessageString() << std::endl;
			return false;
		}
	}

	return false;
}



// 从两个边生成中间线
TopoDS_Edge ComputeMiddleEdge(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2) {
	// 提取两条边的几何曲线和参数范围
	Standard_Real f1, l1, f2, l2;
	Handle(Geom_Curve) curve1 = BRep_Tool::Curve(edge1, f1, l1);
	Handle(Geom_Curve) curve2 = BRep_Tool::Curve(edge2, f2, l2);

	if (curve1.IsNull() || curve2.IsNull()) {
		throw std::invalid_argument("One or both edges have invalid curves.");
	}

	// 取两条曲线的最小公共参数范围
	Standard_Real f = std::max(f1, f2);
	Standard_Real l = std::min(l1, l2);

	if (f >= l) {
		throw std::invalid_argument("The edges do not overlap in parameter space.");
	}

	// 采样曲线点，计算中间点
	const int numSamples = 50; // 采样点数
	TColgp_Array1OfPnt middlePoints(1, numSamples);
	for (int i = 0; i < numSamples; ++i) {
		Standard_Real param = f + i * (l - f) / (numSamples - 1);
		gp_Pnt p1, p2;
		curve1->D0(param, p1);
		curve2->D0(param, p2);

		// 计算中间点
		gp_Pnt midPoint(
			(p1.X() + p2.X()) / 2.0,
			(p1.Y() + p2.Y()) / 2.0,
			(p1.Z() + p2.Z()) / 2.0
		);
		middlePoints.SetValue(i + 1, midPoint);
	}

	// 用中间点生成 B-spline 曲线
	GeomAPI_PointsToBSpline bsplineBuilder(middlePoints);
	Handle(Geom_BSplineCurve) middleCurve = bsplineBuilder.Curve();

	// 用 B-spline 曲线生成边
	return BRepBuilderAPI_MakeEdge(middleCurve).Edge();
}


// 提取圆弧的关键信息
bool ExtractEdgeInfo(const TopoDS_Edge& edge, point_t&startPoint, point_t&endPoint, point_t& centerPoint, Standard_Real& radius,
	Standard_Real& startAngle, Standard_Real& endAngle,int &edgeType) {
	// 提取几何曲线
	Standard_Real first, last;
	gp_Pnt start, end, center;
	Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

	if (curve.IsNull()) {
		std::cerr << "Failed to extract curve from edge." << std::endl;
		return false;
	}

	// 检查几何曲线的类型
	if (curve->IsKind(STANDARD_TYPE(Geom_Line))) {
		edgeType = 0;
		TopoDS_Vertex v1 = TopExp::FirstVertex(edge);
		TopoDS_Vertex v2 = TopExp::LastVertex(edge);
		start = BRep_Tool::Pnt(v1);
		point_t tempStart(start.X(), start.Y());
		startPoint = tempStart;
		end = BRep_Tool::Pnt(v2);
		point_t tempend(end.X(), end.Y());
		endPoint = tempend;

	}
	else if (curve->IsKind(STANDARD_TYPE(Geom_Circle))) {
		edgeType = 1;
		// 检查是否是圆弧
		Handle(Geom_TrimmedCurve) trimmedCurve = Handle(Geom_TrimmedCurve)::DownCast(curve);
		Handle(Geom_Circle) circle;


		if (!trimmedCurve.IsNull()) {
			circle = Handle(Geom_Circle)::DownCast(trimmedCurve->BasisCurve());
		}
		else {
			circle = Handle(Geom_Circle)::DownCast(curve);
		}

		if (circle.IsNull()) {
			std::cerr << "The edge is not a circular arc." << std::endl;
			return false;
		}

		// 获取圆的信息
		gp_Circ circ = circle->Circ();
		center = circ.Location();  // 圆心
		point_t tempCenter(center.X(), center.Y());
		centerPoint = tempCenter;
		radius = circ.Radius();            // 半径

		// 计算起始和终止角度（基于参数范围）
		gp_Dir xDir = circ.XAxis().Direction(); // 圆的X方向
		gp_Dir yDir = circ.YAxis().Direction(); // 圆的Y方向

		gp_Pnt startPoint, endPoint;
		trimmedCurve->D0(first, startPoint);
		trimmedCurve->D0(last, endPoint);

		gp_Vec startVec(center, startPoint);
		gp_Vec endVec(center, endPoint);

		startAngle = atan2(startVec.Dot(yDir), startVec.Dot(xDir)); // 起始角度
		endAngle = atan2(endVec.Dot(yDir), endVec.Dot(xDir));       // 终止角度

		// 将角度归一化到 [0, 2π]
		if (startAngle < 0) startAngle += 2 * M_PI;
		if (endAngle < 0) endAngle += 2 * M_PI;

	}
	else if (curve->IsKind(STANDARD_TYPE(Geom_BSplineCurve))) {
		return false;
	}
	else if (curve->IsKind(STANDARD_TYPE(Geom_TrimmedCurve))) {
		return false;
	}
	else {
		return false;
	}



	return true;
}


bool ExtractLineInfo(const TopoDS_Edge& edge, gp_Pnt& startPoint, gp_Pnt& endPoint) 
{
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge);
	TopoDS_Vertex v2 = TopExp::LastVertex(edge);
	startPoint = BRep_Tool::Pnt(v1);
	endPoint = BRep_Tool::Pnt(v2);

	return true;
}



TopoDS_Edge SheetFlattenProcess::translateEdge(const TopoDS_Edge& edge, const gp_Trsf& translation) {
	// 获取边的几何曲线
	Standard_Real first, last;
	Handle(Geom_Curve) curve = BRep_Tool::Curve(edge, first, last);

	if (curve.IsNull()) {
		std::cerr << "Error: The edge does not have a valid curve." << std::endl;
		return TopoDS_Edge(); // 返回空的 Edge
	}

	try {
		// 对曲线应用平移
		Handle(Geom_Curve) movedCurve = Handle(Geom_Curve)::DownCast(curve->Transformed(translation));

		if (movedCurve.IsNull()) {
			std::cerr << "Error: Transformation failed on curve." << std::endl;
			return TopoDS_Edge(); // 返回空的 Edge
		}

		// 构造一个新的边，保持原始参数
		TopoDS_Edge movedEdge = BRepBuilderAPI_MakeEdge(movedCurve, first, last);
		return movedEdge;
	}
	catch (const Standard_Failure& e) {
		std::cerr << "Exception occurred: " << e.GetMessageString() << std::endl;
		return TopoDS_Edge(); // 返回空的 Edge
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
//		// 使用XCAFDoc_ColorType::XCAFDoc_ColorCurves设定边的颜色
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


	theTranslation.SetTranslation(translationVec);  // 设置平移矩阵

	return;
}
//找到线段的另一端
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
			aLapelEdge1 = aLapeEdges[0];//老二维
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
			

			aOldLapelInterEdge1 = findAdjEdge(aLapelEdge1, pEdgedata1->getVector_interseEdge_2d(), aOldPoint1);//老二维
			aOldLapelInterEdge2 = findAdjEdge(aLapelEdge2, pEdgedata2->getVector_interseEdge_2d(), aOldPoint2);

			quiryEdgeData(aOldLapelInterEdge1, pInterEdgedata1);
			pInterEdgedata1->getNewEdge_2d(aNewLapelInterEdge1);

			quiryEdgeData(aOldLapelInterEdge2, pInterEdgedata2);
			pInterEdgedata2->getNewEdge_2d(aNewLapelInterEdge2);


			pEdgedata1->getNewEdge_2d(aLapelNewEdge1);
			pEdgedata2->getNewEdge_2d(aLapelNewEdge2);


			IsEdgesAtIntersection(aBaseEdge, aLapelEdge1, aInterPoint);
			calTranslateOriention(aBaseEdge, aInterPoint, aTranslation, theDistance);
			aTranslateLaterEdge1 = translateEdge(aLapelNewEdge1, aTranslation);
			aTranslateLaterEdge2 = translateEdge(aLapelNewEdge2, aTranslation);


			FindIntersectionAndExtend(aTranslateLaterEdge1, aTranslateLaterEdge2, aInterPoint);

			FindIntersectionAndExtend(aTranslateLaterEdge1, aNewLapelInterEdge1, aInterPoint);
			FindIntersectionAndExtend(aTranslateLaterEdge2, aNewLapelInterEdge2, aInterPoint);
			/*TrimEdgesAtIntersection1(aTranslateLaterEdge1, aTranslateLaterEdge2, aInterPoint);

			TrimEdgesAtIntersection1(aTranslateLaterEdge1, aNewLapelInterEdge1, aInterPoint);
			TrimEdgesAtIntersection1(aTranslateLaterEdge2, aNewLapelInterEdge2, aInterPoint);*/

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

//求交点坐标
bool SheetFlattenProcess::IsEdgesAtIntersection(TopoDS_Edge& edge1, TopoDS_Edge& edge2, gp_Pnt& intersection) {
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
bool SheetFlattenProcess::SegmentsOverlap(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, TopoDS_Edge& newEdge1, TopoDS_Edge& newEdge2,
	gp_Pnt& newPoint1, gp_Pnt& newPoint2, gp_Pnt& startPoint1, gp_Pnt& startPoint2, gp_Vec& directionVec1, gp_Vec& directionVec2) {
	gp_Pnt basePoint;
	if (AreEdgesSameEndpoints(edge1, edge2))
	{
		return false;
	}
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


//通过交点找交边
TopoDS_Edge SheetFlattenProcess::findAdjEdge(const TopoDS_Edge &baseEdge,const vector<TopoDS_Edge> &interEdges, const gp_Pnt &basePoint)
{
	
		for (auto it : interEdges)
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

// 函数：求点到直线的垂足交点
gp_Pnt SheetFlattenProcess::GetPerpendicularFootPoint(const gp_Pnt& point, const TopoDS_Edge& edge)
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
						oldTarEdge1_2d = findAdjEdge(it.getOldEdge_2d(),it.getVector_interseEdge_2d(), aInterPoint2);//交线里找
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
						IsEdgesAtIntersection(oldTarEdge1_2d, oldTarEdge2_2d, aPoint1);//找到交点
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
	//				oldTarEdge1_2d = findAdjEdge(it.second, aInterPoint2);//交线里找
	//				oldTarEdge2_2d = findAdjEdge(item.second, aInterPoint1);
	//				m_overlapEdges.insert(oldTarEdge1_2d);
	//				m_overlapEdges.insert(oldTarEdge2_2d);

	//				gp_Pnt aPoint1, aPoint2;
	//				IsEdgesAtIntersection(oldTarEdge1_2d, oldTarEdge2_2d, aPoint1);//找到交点
	//				aPoint2 = aPoint1;
	//				// 缩放单位向量，根据给定的距离生成位移向量

	//				double distance = 3.;
	//				gp_Vec displacement1 = aDirvec1.Scaled(distance);
	//				// 将初始点 p1 沿着方向向量移动，生成新的点
	//				aPoint1 = aPoint1.Translated(displacement1);//移动后的交点
	//				gp_Pnt vecticalPoint1 = GetPerpendicularFootPoint(aPoint1, tempEdge1);//垂线交点
	//				newTarEdge1 = BRepBuilderAPI_MakeEdge(aPoint1, vecticalPoint1);
	//				tempEdge1 = BRepBuilderAPI_MakeEdge(aStartPoint1, vecticalPoint1);


	//				gp_Vec displacement2 = aDirvec2.Scaled(distance);
	//				// 将初始点 p1 沿着方向向量移动，生成新的点
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
int SheetFlattenProcess::GetPointSide(const gp_Pnt& baseStart, const gp_Pnt& baseEnd, const gp_Pnt& point) {
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
gp_Vec SheetFlattenProcess::CalMoveVector(const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p) {
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
void SheetFlattenProcess::FindEdgesOnBothSides(vector<SheetFlattenEdgeData>& edges, SheetFlattenEdgeData& baseEdge,
	std::vector<TopoDS_Edge>& leftEdges, std::vector<TopoDS_Edge>& rightEdges, std::vector<TopoDS_Edge>& colines, std::vector<TopoDS_Edge>& otherlines) {
	// 获取基准边的起点和终点
	//TopoDS_Edge aBaseEdge = baseEdge.getOldEdge_2d();
	TopoDS_Edge aBaseEdge = baseEdge.getNewEdge_2d();
	//baseEdge.getVector_newEdge_2d(aBaseEdge);
	TopoDS_Vertex v1 = TopExp::FirstVertex(aBaseEdge);  // 获取起点
	TopoDS_Vertex v2 = TopExp::LastVertex(aBaseEdge);   // 获取终点

	gp_Pnt baseStart = BRep_Tool::Pnt(v1);  // 获取起点坐标
	gp_Pnt baseEnd = BRep_Tool::Pnt(v2);

	// 遍历模型中的所有边
	for (auto &item : edges) {
		//TopoDS_Edge currentEdge = item.getOldEdge_2d();//旧二维

		TopoDS_Edge currentEdge = item.getNewEdge_2d();//旧二维
		//item.getNewEdge_2d(currentEdge);
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
			if (vec1.IsEqual(vec2, 1e-4, 1e-4))
			{
				m_baseVec = CalMoveVector(baseStart, baseEnd, startPnt);
			}
		}
		sideStart = GetPointSide(baseStart, baseEnd, startPnt);
		sideEnd = GetPointSide(baseStart, baseEnd, endPnt);
		// 如果两个端点都在同一侧
		if ((sideStart >= 0 && sideEnd > 0) || ((sideStart > 0 && sideEnd >= 0))) {
			rightEdges.push_back(item.getOldEdge_2d());  // 在右侧
		}
		else if ((sideStart <= 0 && sideEnd < 0) || ((sideStart < 0 && sideEnd <= 0))) {
			leftEdges.push_back(item.getOldEdge_2d()); // 在左侧
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


TopoDS_Edge SheetFlattenProcess::TranslatePositiveEdge(const TopoDS_Edge& edge, const gp_Trsf& translation, TopoDS_Edge& theNewEdge1, TopoDS_Edge& theNewEdge2)
{
	TopoDS_Vertex v1 = TopExp::FirstVertex(edge);  // 获取起点
	TopoDS_Vertex v2 = TopExp::LastVertex(edge);   // 获取终点
	gp_Vec translationVec = translation.TranslationPart();
	gp_Pnt p1 = BRep_Tool::Pnt(v1);  // 获取起点坐标
	gp_Pnt p2 = BRep_Tool::Pnt(v2);

	gp_Pnt old_p1 = p1;
	gp_Pnt old_p2 = p2;


	// 5. 对边的两个端点应用平移
	p1.Transform(translation);  // 移动起点
	p2.Transform(translation);  // 移动终点
	theNewEdge1 = BRepBuilderAPI_MakeEdge(old_p1, p1);
	theNewEdge2 = BRepBuilderAPI_MakeEdge(old_p2, p2);
	// 6. 使用移动后的点构造新的边
	TopoDS_Edge newEdge = BRepBuilderAPI_MakeEdge(p1, p2);

	return newEdge;
}

bool SheetFlattenProcess::calTranslate(SheetFlattenEdgeData& theEdge, const TopoDS_Face& theFace,gp_Vec & theVec)
{
	gp_Trsf translation;
	TopoDS_Edge a2dBaseEdge, a2dTarEdge, a2dNewEdge, a2dInterEdge;
	TopoDS_Edge newEdge;
	gp_Pnt p1, p2, intersection;
	bool isReduct = true;
	a2dBaseEdge = theEdge.getOldEdge_2d();

	TopoDS_Vertex v1 = TopExp::FirstVertex(a2dBaseEdge);  // 获取起点
	TopoDS_Vertex v2 = TopExp::LastVertex(a2dBaseEdge);   // 获取终点
	gp_Pnt baseStart = BRep_Tool::Pnt(v1);  // 获取起点坐标
	gp_Pnt baseEnd = BRep_Tool::Pnt(v2);  // 获取终点坐标


	gp_Vec normalVec;

	vector<int> result;
	for (TopExp_Explorer explorer(theFace, TopAbs_EDGE); explorer.More(); explorer.Next()) {
		const TopoDS_Edge& currentEdge = TopoDS::Edge(explorer.Current());
		SheetFlattenEdgeData* pCurrentEdgedata;
		quiry3dEdgeData(currentEdge, pCurrentEdgedata);
		a2dTarEdge = pCurrentEdgedata->getOldEdge_2d();
		if (a2dBaseEdge == a2dTarEdge)
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
			result.emplace_back(sideStart);
		}
		if (sideEnd != 0)
		{
			normalVec = CalMoveVector(baseStart, baseEnd, endPnt);
			result.emplace_back(sideEnd);
		}

	}

	theVec = normalVec;

	return std::all_of(result.begin(), result.end(), [&result](int value) {
		return value == result[0]; // 比较每个元素与第一个元素
		});

}
//包角时朝着向量方向移动边,裁剪边
TopoDS_Edge SheetFlattenProcess::calRetractTranslate(SheetFlattenEdgeData & theEdge, const TopoDS_Face& theFace, const double theDistance, double theAngle) {
	// 1. 获取边的两个端点
	gp_Trsf translation;
	TopoDS_Edge a2dBaseEdge, a2dTarEdge, a2dNewEdge, a2dInterEdge;
	TopoDS_Edge newEdge;
	gp_Pnt p1, p2, intersection;
	bool isReduct = true;
	a2dBaseEdge = theEdge.getOldEdge_2d();
	theEdge.getNewEdge_2d(a2dNewEdge);
	TopoDS_Vertex w1 = TopExp::FirstVertex(a2dNewEdge);  // 获取起点
	TopoDS_Vertex w2 = TopExp::LastVertex(a2dNewEdge);   // 获取终点

	p1 = BRep_Tool::Pnt(w1);  // 获取起点坐标
	p2 = BRep_Tool::Pnt(w2);

	TopoDS_Vertex v1 = TopExp::FirstVertex(a2dBaseEdge);  // 获取起点
	TopoDS_Vertex v2 = TopExp::LastVertex(a2dBaseEdge);   // 获取终点

	gp_Pnt baseStart = BRep_Tool::Pnt(v1);  // 获取起点坐标
	gp_Pnt baseEnd = BRep_Tool::Pnt(v2);  // 获取终点坐标


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

	newEdge = translateEdge(a2dNewEdge, translation);
	// 5. 对边的两个端点应用平移
	//p1.Transform(translation);  // 移动起点
	//p2.Transform(translation);  // 移动终点

	// 6. 使用移动后的点构造新的边
	//newEdge = BRepBuilderAPI_MakeEdge(p1, p2);
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
			vector<TopoDS_Edge> aBaseEdges,aTarEdges;
			//if (pEdgedata->getVector_newEdge_2d(aEdges))
			{
				if (!pEdgedata->getVector_newEdge_2d(aBaseEdges))
				{
					pEdgedata->setVector_newEdge_2d(aBaseEdges);
				}
				if (!theEdge.getVector_newEdge_2d(aTarEdges))
				{
					theEdge.setVector_newEdge_2d(aTarEdges);
				}
				//pEdgedata->getVector_WrapAngleEdges(aBaseEdges);
				if (pEdgedata->isBendEdge())
				{
					continue;
				}
				//if (!isReduct)
				{
					a2dInterEdge = aBaseEdges.back();
					//TrimEdgesAtIntersection1(newEdge, a2dInterEdge, intersection);
					FindIntersectionAndExtend(newEdge, a2dInterEdge, intersection);

					if (pEdgedata->isAddWrapAngleEdge())
					{
						aBaseEdges[aBaseEdges.size() - 1] = a2dInterEdge;
						pEdgedata->setVector_newEdge_2d(aBaseEdges);
					}
					else
					{
						pEdgedata->insertEdgeTo_new2d(a2dInterEdge);
						pEdgedata->setAddWrapAngleEdge(true);
					}
					//aBaseEdges[aBaseEdges.size() - 1] = a2dInterEdge;
					//pEdgedata->setVector_WrapAngleEdges(aBaseEdges);
					if (theEdge.isAddWrapAngleEdge())
					{
						aTarEdges[aTarEdges.size() - 1] = newEdge;
						theEdge.setVector_newEdge_2d(aTarEdges);
					}
					else
					{
						theEdge.insertEdgeTo_new2d(newEdge);
						theEdge.setAddWrapAngleEdge(true);
					}



				}
			}
		}
	}
	//theEdge.insertEdgeTo_new2d(newEdge);
	return newEdge;  // 返回新的边


}
//朝着向量方向移动边
void SheetFlattenProcess::calTranslate(const double distance, gp_Trsf& translation, const double& angle) {
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
TopoDS_Edge SheetFlattenProcess::midleLine(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2)
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
bool SheetFlattenProcess::TrimEdgesAtIntersection1(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection) {
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
bool SheetFlattenProcess::TrimEdgesAtIntersection(TopoDS_Edge& theEdge1, TopoDS_Edge& theEdge2, gp_Pnt& theIntersection) {
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


// 判断两个线段是否相交
bool AreEdgesIntersecting(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, gp_Pnt& intersectionPoint) {
	// 提取曲线和参数范围
	Standard_Real f1, l1, f2, l2;
	Handle(Geom_Curve) curve1 = BRep_Tool::Curve(edge1, f1, l1);
	Handle(Geom_Curve) curve2 = BRep_Tool::Curve(edge2, f2, l2);

	if (curve1.IsNull() || curve2.IsNull()) {
		std::cerr << "One or both edges have invalid curves." << std::endl;
		return false;
	}


	// 使用 GeomAPI_ExtremaCurveCurve 获取最近点对
	GeomAPI_ExtremaCurveCurve extrema(curve1, curve2);
	if (extrema.NbExtrema() == 0) {
		return false; // 没有交点
	}

	Standard_Real tolerance = Precision::Confusion();

	for (int i = 1; i <= extrema.NbExtrema(); ++i) {
		try {
			// 判断最近点对的距离是否接近 0
			if (extrema.Distance(i) < tolerance) {
				gp_Pnt p1, p2;
				extrema.Points(i, p1, p2);

				// 检查交点是否在两条线段的参数范围内
				Standard_Real param1, param2;
				extrema.Parameters(i, param1, param2);

				if (param1 >= f1-1e-7 && param1 <= l1+1e-7 && param2 >= f2-1e-7 && param2 <= l2+1e-7) {
					intersectionPoint = p1; // 确认交点
					return true;
				}
			}
		}
		catch (Standard_Failure& err) {
			std::cerr << "Error while checking intersection: " << err.GetMessageString() << std::endl;
			return false;
		}
	}

	return false; // 没有交点
}


//判断两条线段相交
bool SheetFlattenProcess::EdgeIntersect(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2)
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
	vector<TopoDS_Edge> intersectionLines, unintersectionLines;
	unordered_set<TopoDS_Edge> lines;
	gp_Pnt point;
	for (auto &elem : m_EdgeData)
	{
		for (auto &item : elem)
		{
			auto faces = item.getVector_face();
			for (auto &it : faces)
			{


				for (TopExp_Explorer edgeExplorer(it, TopAbs_EDGE); edgeExplorer.More(); edgeExplorer.Next())
				{

					TopoDS_Edge otherEdge = TopoDS::Edge(edgeExplorer.Current());//三维的线
					if (otherEdge != item.getEdge_3d())
					{
						gp_Pnt intersection1;
						SheetFlattenEdgeData* pEdgedata;
						quiry3dEdgeData(otherEdge, pEdgedata);
						TopoDS_Edge twoEdge = pEdgedata->getOldEdge_2d();
						//TopoDS_Edge twoEdge = m_ThreeToTwoEdge.find(otherEdge)->second;//旧二维
						if (AreEdgesIntersecting(twoEdge, item.getOldEdge_2d(), point))/*AreEdgesIntersecting(twoEdge, ThreeToTwoEdge.find(edge)->second)*/
						{
							intersectionLines.emplace_back(twoEdge);
						}
						else {
							unintersectionLines.emplace_back(twoEdge);
						}
						lines.insert(twoEdge);
					}
				}

			}
			item.setVector_sameFace_interseEdge_2d(intersectionLines);
			item.setVector_sameFaceEdge_uninterseEdge_2d(unintersectionLines);
			item.setVector_sameFaceEdge_2d(lines);
			intersectionLines.clear();
			unintersectionLines.clear();
			lines.clear();

		}
	}
}

void SheetFlattenProcess::allReation2dEdge()
{
	vector<TopoDS_Edge> interseEdges;
	gp_Pnt interPoint;
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
					if (!isOverlap(baseEdge, tarEdge) && AreEdgesIntersecting(baseEdge, tarEdge, interPoint))
					{
						interseEdges.push_back(tarEdge);
						
					}
				}
			}
			base.setVector_interseEdge_2d(interseEdges);
			interseEdges.clear();
		}
	}
}
void SheetFlattenProcess::allReation2dNewEdge()
{
	TopoDS_Edge baseEdge, tarEdge;
	vector<TopoDS_Edge> interseEdges;
	gp_Pnt insterPoint;
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
					if (AreEdgesIntersecting(baseEdge, tarEdge,insterPoint) && !isOverlap(baseEdge, tarEdge))
					{
						interseEdges.push_back(tar.getOldEdge_2d());
					}
				}
			}
			base.setVector_interseEdge_new2d(interseEdges);
			interseEdges.clear();
		}
	}
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
			gp_Pnt interPoint;
			if (!AreEdgesIntersecting(newInterseEdge, newbaseEdge, interPoint))
			{
				continue;
			}
			//TrimEdgesAtIntersection1(newbaseEdge, newInterseEdge, intersection);//延长线到交点
			FindIntersectionAndExtend(newbaseEdge, newInterseEdge, intersection); 
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

		//TrimEdgesAtIntersection1(newbaseEdge, newInterseEdge, intersection);//延长线到交点
		FindIntersectionAndExtend(newbaseEdge, newInterseEdge, intersection);

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

			//TrimEdgesAtIntersection1(newbaseEdge, newInterseEdge, intersection);//延长线到交点
			FindIntersectionAndExtend(newbaseEdge, newInterseEdge, intersection);
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
// 函数计算两面夹角
bool SheetFlattenProcess::CalculateAngleBetweenFaces(const TopoDS_Face& theBaseFace, const TopoDS_Face& theTarFace, double& angle) {

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
bool SheetFlattenProcess::CalculateAngleBetweenFaces(const vector<TopoDS_Face>& face, double& angle) {

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
				check();
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
							if (!pTarEdgadata->isBendEdge())
							{
								if (pTarEdgadata->isOutline())
								{
									aTarEdge = pTarEdgadata->getOutlineEdge();
								}
								else {
									pTarEdgadata->getNewEdge_2d(aTarEdge);
								}
								gp_Pnt intersection;
								//TrimEdgesAtIntersection1(aBaseEdge, aTarEdge, intersection);

								FindIntersectionAndExtend(aBaseEdge, aTarEdge, intersection);
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

void SheetFlattenProcess::ToInfo_DXF(const TopoDS_Edge& edge, point_t& startPoint, point_t& endPoint, point_t& center, Standard_Real& radius,
	Standard_Real& startAngle, Standard_Real& endAngle, int& edgeType)
{
	ExtractEdgeInfo(edge, startPoint, endPoint, center, radius, startAngle, endAngle, edgeType);
	if (edgeType == LineCurve)
	{
		m_line.emplace_back(startPoint, endPoint, "0");
	}
	else if (edgeType == CircleCurve)
	{
		m_eclipse.emplace_back(center, radius, startAngle, endAngle, "0");
	}
}

void SheetFlattenProcess::check()
{
	TopoDS_Compound aCompound;
	BRep_Builder aBuilder;
	aBuilder.MakeCompound(aCompound);
	BRepBuilderAPI_MakeWire wireBuilder;
	point_t startPoint, endPoint, center;
	Standard_Real radius, startAngle, endAngle;
	int edgeType = -1;
	m_line.clear();
	m_circle.clear();
	m_eclipse.clear();
	for (auto elem : m_EdgeData)
	{
		for (auto item : elem)
		{
			TopoDS_Edge edge;
			vector<TopoDS_Edge> aEdges;
			CurveType type = item.GetCurveType();
			if (item.getOverlapeOutlineEdge(edge))
			{
				if (edge.IsNull()) {
					std::cerr << "Invalid edge detected!" << std::endl;
				}
				ToInfo_DXF(edge, startPoint, endPoint, center, radius, startAngle, endAngle, edgeType);
				aBuilder.Add(aCompound, edge);
				//wireBuilder.Add(edge);
			}
			if (item.isOutline())
			{
				edge = item.getOutlineEdge();
				if (edge.IsNull()) {
					std::cerr << "Invalid edge detected!" << std::endl;
				}
				ToInfo_DXF(edge, startPoint, endPoint, center, radius, startAngle, endAngle, edgeType);
				aBuilder.Add(aCompound, edge);
				//wireBuilder.Add(edge);
			}
			if (item.getVector_newEdge_2d(aEdges))
			{
				for (auto it : aEdges)
				{
					ToInfo_DXF(it, startPoint, endPoint, center, radius, startAngle, endAngle, edgeType);
					aBuilder.Add(aCompound, it);
					//wireBuilder.Add(it);
				
				}
			}
			else
			{
				for (auto it : aEdges)
				{
					ToInfo_DXF(it, startPoint, endPoint, center, radius, startAngle, endAngle, edgeType);
					aBuilder.Add(aCompound, it);
					//wireBuilder.Add(it);

				}
				/*edge = aEdges[0];
				if (edge.IsNull()) {
					std::cerr << "Invalid edge detected!" << std::endl;
				}
				aBuilder.Add(aCompound, edge);*/
				//wireBuilder.Add(edge);
			}
		}
		//FlattenFace::BuildDocStd(aCompound);
		if (wireBuilder.IsDone())
		{
			//TopoDS_Wire wire = wireBuilder.Wire();
			//aBuilder.Add(aCompound, wire);
		}
		//wireBuilder = BRepBuilderAPI_MakeWire();
	}
	FlattenFace::BuildDocStd(aCompound);
}


double CalculateEdgeDistance(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2) {
	// 提取两条边的几何曲线
	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1_1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p1_2 = BRep_Tool::Pnt(v1_2);

	// 获取第二条边的两个端点
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

// 方法二：基于单位向量判断方向一致
bool AreVectorsSameDirectionNormalized(const gp_Vec& vec1, const gp_Vec& vec2) {
	gp_Vec normVec1 = vec1.Normalized();
	gp_Vec normVec2 = vec2.Normalized();
	double diff = (normVec1 - normVec2).Magnitude();
	return diff < 1e-6; // 使用小误差值判断相等
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

				TopoDS_Edge newedge = translateEdge(it.getOldEdge_2d(), translation);
				changeMapIdex(it.getOldEdge_2d(), newedge);
				it.setOldEdge_2d(newedge);
			}
		}
		//check();
	}
}

// 计算两条线段的顺时针夹角
double CalculateEdgeClockwiseAngle(const TopoDS_Edge & edge1, const TopoDS_Edge & edge2,const gp_Dir& referenceAxis) {
	// 计算两线段的方向向量

	TopoDS_Vertex v1_1 = TopExp::FirstVertex(edge1);
	TopoDS_Vertex v1_2 = TopExp::LastVertex(edge1);
	gp_Pnt p1 = BRep_Tool::Pnt(v1_1);
	gp_Pnt p2 = BRep_Tool::Pnt(v1_2);

	// 获取第二条边的两个端点
	TopoDS_Vertex v2_1 = TopExp::FirstVertex(edge2);
	TopoDS_Vertex v2_2 = TopExp::LastVertex(edge2);
	gp_Pnt q1 = BRep_Tool::Pnt(v2_1);
	gp_Pnt q2 = BRep_Tool::Pnt(v2_2);

	gp_Vec vec1(p1, p2);
	gp_Vec vec2(q1, q2);

	// 计算标准夹角
	double dotProduct = vec1 * vec2;
	double magnitudeProduct = vec1.Magnitude() * vec2.Magnitude();
	double angle = acos(dotProduct / magnitudeProduct); // 返回值在 [0, π]

	// 使用叉乘判断方向
	gp_Vec crossProduct = vec1.Crossed(vec2); // vec1 × vec2
	double direction = crossProduct * gp_Vec(referenceAxis); // 点积判断方向

	// 如果方向为负，则夹角是顺时针，否则是逆时针
	if (direction < 0) {
		angle = 2 * M_PI - angle; // 转为顺时针方向
	}

	return angle; // 返回值范围 [0, 2π]
}



// 计算两个向量之间的顺时针夹角（以轴为参考）
double CalculateClockwiseAngle(const gp_Vec& vec1, const gp_Vec& vec2, const gp_Dir& referenceAxis) {
	// 计算标准夹角
	double dotProduct = vec1 * vec2;
	double magnitudeProduct = vec1.Magnitude() * vec2.Magnitude();
	//double angle = acos(dotProduct / magnitudeProduct); // 返回值在 [0, π]
	double angle = std::atan2(vec1.Crossed(vec2).Magnitude(), vec1.Dot(vec2));
	double baseAngle = angle;
	// 使用叉乘判断方向
	gp_Vec crossProduct = vec1.Crossed(vec2); // vec1 × vec2
	double direction = crossProduct * gp_Vec(referenceAxis); // 点积判断方向

	// 如果方向为负，则夹角是顺时针，否则是逆时针
	if (direction < 0) {
		angle = 2 * M_PI - angle; // 转为顺时针方向
	}
	if (angle == M_PI)
	{
		return 0.;
	}
	else if (fabs(angle - 0.) < 1e-2)
	{
		return M_PI;
	}
	else if (angle > M_PI)
	{
		angle =3* M_PI - angle;
	}
	else if (angle < M_PI)
	{
		angle = M_PI - angle;
	}

	return angle; // 返回值范围 [0, 2π]
}

// 将一条线以原点为中心旋转一定角度
TopoDS_Edge RotateEdge(const TopoDS_Edge& edge, double angleInDegrees) {
	// 将角度从度转换为弧度
	double angleInRadians = angleInDegrees;// * M_PI / 180.0;

	// 定义旋转轴（以原点为中心，沿 Z 轴旋转）
	gp_Ax1 rotationAxis(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));

	// 创建旋转变换
	gp_Trsf rotationTransform;
	rotationTransform.SetRotation(rotationAxis, angleInRadians);

	// 应用变换到边
	BRepBuilderAPI_Transform transformer(edge, rotationTransform);
	return TopoDS::Edge(transformer.Shape());
}

bool AreEdgesParallel(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, double tolerance = Precision::Confusion()) {
	// 获取两条边的几何曲线
	Standard_Real f1, l1, f2, l2;
	Handle(Geom_Curve) curve1 = BRep_Tool::Curve(edge1, f1, l1);
	Handle(Geom_Curve) curve2 = BRep_Tool::Curve(edge2, f2, l2);

	if (curve1.IsNull() || curve2.IsNull()) {
		// 如果任何一条边没有几何曲线，返回 false
		return false;
	}

	// 计算两条边的方向向量
	gp_Vec dir1, dir2;
	gp_Pnt p1_start, p1_end, p2_start, p2_end;

	curve1->D0(f1, p1_start);
	curve1->D0(l1, p1_end);
	dir1 = gp_Vec(p1_start, p1_end);

	curve2->D0(f2, p2_start);
	curve2->D0(l2, p2_end);
	dir2 = gp_Vec(p2_start, p2_end);

	// 归一化方向向量
	if (dir1.Magnitude() < tolerance || dir2.Magnitude() < tolerance) {
		// 如果某条边的长度接近于零，认为不平行
		return false;
	}

	dir1.Normalize();
	dir2.Normalize();

	// 检查向量是否平行（点积的绝对值接近1）
	double dotProduct = Abs(dir1.Dot(dir2));
	return Abs(dotProduct - 1.0) < tolerance;
}

// 平移第二条 Edge 到第一条 Edge 的位置
bool AlignEdges(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2, gp_Trsf &trsf) {
	// 提取第一条 Edge 的起点和终点
	Standard_Real firstParam1, lastParam1;
	Handle(Geom_Curve) curve1 = BRep_Tool::Curve(edge1, firstParam1, lastParam1);
	if (curve1.IsNull()) return false; // 如果曲线为空，返回 false
	gp_Pnt startPnt1, endPnt1;
	curve1->D0(firstParam1, startPnt1);
	curve1->D0(lastParam1, endPnt1);

	// 提取第二条 Edge 的起点和终点
	Standard_Real firstParam2, lastParam2;
	Handle(Geom_Curve) curve2 = BRep_Tool::Curve(edge2, firstParam2, lastParam2);
	if (curve2.IsNull()) return false; // 如果曲线为空，返回 false
	gp_Pnt startPnt2, endPnt2;
	curve2->D0(firstParam2, startPnt2);
	curve2->D0(lastParam2, endPnt2);

	// 计算两条线段的方向向量并归一化
	gp_Vec dir1(startPnt1, endPnt1);
	gp_Vec dir2(startPnt2, endPnt2);

	if (dir1.Magnitude() == 0 || dir2.Magnitude() == 0) return false; // 避免零长度线段
	dir1.Normalize();
	dir2.Normalize();


	gp_Vec translationVector;


	// 计算点积判断方向是否一致
	Standard_Real dotProduct = dir1 * dir2;
	if (dotProduct > 0.999)
	{
		gp_Vec tempRanslationVector(startPnt2, startPnt1);
		translationVector = tempRanslationVector;
	}
	else
	{
		gp_Vec tempRanslationVector(endPnt2, startPnt1);
		translationVector = tempRanslationVector;
	}
	// 构建平移变换
	trsf.SetTranslation(translationVector);


	return true;
}


void SheetFlattenProcess::processSplitEdge()
{
	double maxDistance = -1;
	gp_Dir standard(0, 0, 1);
	gp_Vec moveVec,baseVec,tarVec;
	TopoDS_Edge baseEdge, tarEdge;
	SheetFlattenEdgeData* baseEdgeData = nullptr, * tarEdgeData = nullptr;
	for (auto& baseGroup : m_EdgeData) {
		for (auto& targetGroup : m_EdgeData) {
			// 如果是同一个分组，跳过
			if (&baseGroup == &targetGroup) continue;

			// 遍历基础边
			for (auto& base : baseGroup) {
				// 如果基础边已处理过，跳过
				if (base.isProcessWrapAngle()) continue;

				for (auto& target : targetGroup) {
					// 如果目标边已处理过或两边端点不同，跳过
					if (target.isProcessWrapAngle() ||
						!AreEdgesSameEndpoints(base.getEdge_3d(), target.getEdge_3d())) {
						continue;
					}
	
					// 获取基础边和目标边的面数据
					const auto& baseFaces = base.getVector_face();
					const auto& targetFaces = target.getVector_face();

					// 如果任意边的关联面数量 >= 2，跳过
					if (baseFaces.size() >= 2 || targetFaces.size() >= 2) continue;

					// 计算两边的夹角
					double faceAngle = 0.0;
					CalculateAngleBetweenFaces(baseFaces[0], targetFaces[0], faceAngle);

					// 更新基础边和目标边的数据
					base.setAngle(faceAngle);
					base.setProcessWrapAngle(true);

					target.setProcessWrapAngle(true);
					target.setWrapAngle(true);
					target.setAngle(faceAngle);

					double distance = CalculateEdgeDistance(base.getOldEdge_2d(), target.getOldEdge_2d());

					if (fabs(M_PI - faceAngle) > 1e-6)
					{
						if (distance > maxDistance)
						{
							maxDistance = distance;
							baseEdgeData = &base;
							tarEdgeData = &target;
							baseEdge = base.getOldEdge_2d();
							tarEdge = target.getOldEdge_2d();
						}
					}
				}
				
			}
			if (maxDistance != -1)
			{
				calTranslate(*baseEdgeData, baseEdgeData->getVector_face()[0], baseVec);
				calTranslate(*tarEdgeData, tarEdgeData->getVector_face()[0], tarVec);
				//calTranslate(baseGroup[maxBaseIndex], baseGroup[maxBaseIndex].getVector_face()[0], baseVec);
				//calTranslate(targetGroup[maxTarIndex], targetGroup[maxTarIndex].getVector_face()[0], tarVec);
				double angle = CalculateClockwiseAngle(baseVec, tarVec, standard);

				double teset = fabs(angle - M_PI);
				bool isrotate = false;
				//angle = CalculateEdgeClockwiseAngle(base.getNewEdge_2d(), target.getNewEdge_2d(), standard);
				if (fabs(angle) > 1e-2 )
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
					for (auto& base : baseGroup) {
						// 如果基础边已处理过，跳过
						if (base.isProcessWrapAngle()) //continue;
						for (auto& target : targetGroup) {
							// 如果目标边已处理过或两边端点不同，跳过
							if (!AreEdgesSameEndpoints(base.getEdge_3d(), target.getEdge_3d())) {
								continue;
							}

							gp_Vec transaltion1;
							double distance = CalculateEdgeDistance(base.getOldEdge_2d(), target.getOldEdge_2d());
							if (distance > maxDistance && areEdgesNearlyParallel(base.getOldEdge_2d(), target.getOldEdge_2d(),10.)
								&& calTranslate(target, target.getVector_face()[0], transaltion1))
							{
								maxDistance = distance;
								baseEdgeData = &base;
								tarEdgeData = &target;
								baseEdge = base.getOldEdge_2d();
								tarEdge = target.getOldEdge_2d();
							}

						}
					}
				}
				
				gp_Trsf transaltion;
				baseEdge = baseEdgeData->getOldEdge_2d();
				tarEdge = tarEdgeData->getOldEdge_2d();
				AlignEdges(baseEdge, tarEdge, transaltion);
				for (auto& elem : targetGroup)
				{
					BRepBuilderAPI_Transform transformer(elem.getOldEdge_2d(), transaltion);
					TopoDS_Edge transformedEdge = TopoDS::Edge(transformer.Shape());
					changeMapIdex(elem.getOldEdge_2d(), transformedEdge);
					elem.setOldEdge_2d(transformedEdge);
				}
				check();
				calTranslate(*tarEdgeData, tarEdgeData->getVector_face()[0], moveVec);
				for (auto& elem : targetGroup)
				{
					gp_Vec translationVec = moveVec * 20.;
					gp_Trsf aTranslation;
					aTranslation.SetTranslation(translationVec);
					BRepBuilderAPI_Transform transformer(elem.getOldEdge_2d(), aTranslation);
					TopoDS_Edge transformedEdge = TopoDS::Edge(transformer.Shape());
					changeMapIdex(elem.getOldEdge_2d(), transformedEdge);
					elem.setOldEdge_2d(transformedEdge);
				}
				check();
				//maxDistance = CalculateEdgeDistance(baseGroup[maxBaseIndex].getOldEdge_2d(), targetGroup[maxTarIndex].getOldEdge_2d());
			}
			maxDistance = -1.;
		}
	}
}

void SheetFlattenProcess::processEdges_WrapAngle() {
	// 遍历每组数据
	for (auto& baseGroup : m_EdgeData)
	{
		// 移动尚未完成的边
		for (auto& edge : baseGroup) 
		{
			if (!edge.isBendEdge())
			{
				continue;
			}
			if (!edge.isFinish()) 
			{
				moveEdge(baseGroup, edge);
				//check();
			}
		}
	}
}



void SheetFlattenProcess::generate()
{
	check();
	//processParallelEdge();
	processSplitEdge();
	check();
	//moveSplitEdge();
	//check();
	allReation2dEdge();
	processSameFaceEdgeRelation();
	processOverlap();
	allReation2dNewEdge();
	check();

	processEdges_WrapAngle();
	check();
	
	makeLapelMap();//折边
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
					gp_Pnt interPoint;
					if (AreEdgesIntersecting(aBaseEdge, aTarEdge, interPoint))
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
								FinishEdges.insert(aTarEdge);
								it.setOverlapeOutlineEdge(aNewEdge);
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
	check();
	SaveDxfFile saveFile;
	saveFile.SaveDxf(m_line, m_circle, m_eclipse, "D:\\1_work\\test.dxf");
}

bool AreVectorsNearlyParallel(const gp_Vec& vec1, const gp_Vec& vec2, double angleThresholdDegrees) {
	// 将阈值角度从度数转换为弧度
	double angleThresholdRadians = angleThresholdDegrees * M_PI / 180.0;

	// 计算向量的点积和模
	double dotProduct = vec1.Dot(vec2);
	double magnitudeProduct = vec1.Magnitude() * vec2.Magnitude();

	// 检查零向量情况
	if (magnitudeProduct < 1e-7) {
		return false; // 零向量无法比较方向
	}

	// 计算 cos(theta)
	double cosTheta = dotProduct / magnitudeProduct;

	// 判断是否基本同向
	return cosTheta > std::cos(angleThresholdRadians);
}

void SheetFlattenProcess::generateNewEdge(const vector<TopoDS_Edge>& theMoveEdge, const gp_Trsf& translation, bool theMove)
{
	SheetFlattenEdgeData* aEdgadata = nullptr;
	vector<TopoDS_Edge> aEdges;
	vector<TopoDS_Edge> tempedge;
	gp_Pnt overPoint, oldOtherPoint, otherPoint;
	TopoDS_Edge newEdge;
	for (auto it : theMoveEdge)
	{
		quiryEdgeData(it, aEdgadata);
		aEdgadata->getVector_newEdge_2d(tempedge);
		if (theMove)
		{
			if (aEdgadata->isOverlapeEdge())
			{
				if (!aEdgadata->isFirstMoveOverlapeEdge())
				{
					aEdgadata->setFirstMoveOverlapeEdge(true);
					overPoint = aEdgadata->getOverLapEdge_Point();
					otherPoint = findOtherPoint(aEdgadata->getNewEdge_2d(), overPoint);
					oldOtherPoint = otherPoint;
					gp_Vec baseVec(otherPoint, overPoint);
					otherPoint.Transform(translation);
					gp_Vec tarVec(oldOtherPoint, otherPoint);
					if (AreVectorsNearlyParallel(baseVec, tarVec, 1.))
					{
						newEdge = BRepBuilderAPI_MakeEdge(overPoint, otherPoint);
					}
					else
					{
						aEdgadata->setFirstMoveOverlapeEdge(false);
						overPoint.Transform(translation);
						newEdge = BRepBuilderAPI_MakeEdge(overPoint, otherPoint);
						aEdgadata->setOverLapEdge_Point(overPoint);

					}
					aEdges.emplace_back(newEdge);
					aEdgadata->setVector_newEdge_2d(aEdges);
					aEdges.clear();
					continue;
				}
				else
				{
					gp_Pnt aBasePoint = aEdgadata->getOverLapEdge_Point();
					aBasePoint.Transform(translation);
					aEdgadata->setOverLapEdge_Point(aBasePoint);
					for (auto elem : tempedge)
					{
						TopoDS_Edge newedge = translateEdge(elem, translation);
						aEdges.emplace_back(newedge);
					}
					aEdgadata->setVector_newEdge_2d(aEdges);
					aEdges.clear();
				}
			}
			else
			{
				for (auto elem : tempedge)
				{
					TopoDS_Edge newedge = translateEdge(elem, translation);
					aEdges.emplace_back(newedge);
				}
				aEdgadata->setVector_newEdge_2d(aEdges);
				aEdges.clear();
			}
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
	for (auto it : theMoveEdge)//遍历共线的线
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
				newedge = translateEdge(tempedge, theTranslation_slot);
			}
			else {
				//newedge = TranslateEdge(tempedge, theTranslation);
				newedge = translateEdge(tempedge, theTranslation);
			}
			//TopoDS_Edge midedge = midleLine(tempedge, newedge);
			TopoDS_Edge midedge = ComputeMiddleEdge(tempedge, newedge);
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
	int rightNumber = 0, leftNumber = 0;
	if (edge.isBendEdge())
	{

		edge.setFinish(true);
		gp_Trsf translation, translation_slot;
		Standard_Real angle = edge.getAngle();
		//CalculateAngleBetweenFaces(faces, angle);

		TopoDS_Edge baseEdge = edge.getOldEdge_2d();//旧二维
		FindEdgesOnBothSides(edges, edge, leftEdges, rightEdges, colines, otherlines);//所有容器装的都是旧二维,朝里挪
		

		for (auto& elem : leftEdges)
		{
			SheetFlattenEdgeData* aEdgadata = nullptr;
			quiryEdgeData(elem, aEdgadata);
			if (aEdgadata->isBendEdge())
			{
				++leftNumber;
			}
		}

		for (auto& elem : rightEdges)
		{
			SheetFlattenEdgeData* aEdgadata = nullptr;
			quiryEdgeData(elem, aEdgadata);
			if (aEdgadata->isBendEdge())
			{
				++rightNumber;
			}
		}

		if (rightNumber <= leftNumber)
		{
			for (auto& it : colines)//遍历共线的线
			{
				SheetFlattenEdgeData* aEdgedata = nullptr;
				quiryEdgeData(it, aEdgedata);
				unordered_set<TopoDS_Edge> Edges = aEdgedata->getVector_sameFaceEdge_2d();
				for (auto &item : rightEdges)
				{
					if (Edges.find(item) != Edges.end())
					{
						SheetFlattenEdgeData* aSameFaceEdgeData = nullptr;
						quiryEdgeData(item, aSameFaceEdgeData);
						aSameFaceEdgeData->setTranslateType(true);
					}
				}

			}
		}
		else
		{
			for (auto& it : colines)//遍历共线的线
			{
				SheetFlattenEdgeData* aEdgedata = nullptr;
				quiryEdgeData(it, aEdgedata);
				unordered_set<TopoDS_Edge> Edges = aEdgedata->getVector_sameFaceEdge_2d();
				for (auto& item : leftEdges)
				{
					if (Edges.find(item) != Edges.end())
					{
						SheetFlattenEdgeData* aSameFaceEdgeData = nullptr;
						quiryEdgeData(item, aSameFaceEdgeData);
						aSameFaceEdgeData->setTranslateType(true);
					}
				}

			}
		}


		double moveDis = 2.;
		calTranslate(moveDis, translation_slot, angle);
		calTranslate(moveDis, translation, angle);//确定移动方向和角度
		if (angle > M_PI)
		{
			m_isNegativeBend = false;
		}
		else
		{
			m_isNegativeBend = true;
		}
		if (rightNumber <= leftNumber)
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
		rightNumber = 0;
		leftNumber = 0;
	


	}
	check();
}
//端点是否相同
bool SheetFlattenProcess::AreEdgesSameEndpoints(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2) {
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
	bool test = p1_1.IsEqual(p2_2, Precision::Confusion());
	bool test1 = p1_2.IsEqual(p2_1, Precision::Confusion());
	bool sameStartEnd = p1_1.IsEqual(p2_1, 1.e-4) && p1_2.IsEqual(p2_2, 1.e-4);
	bool sameEndStart = p1_1.IsEqual(p2_2, 1.e-4) && p1_2.IsEqual(p2_1, 1.e-4);

	return sameStartEnd || sameEndStart;
}
#endif