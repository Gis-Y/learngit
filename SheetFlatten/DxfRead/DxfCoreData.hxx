#ifndef _DXF_CORE_DATA_16F4EFB686D642BDB4047C57533CD9D0
#define _DXF_CORE_DATA_16F4EFB686D642BDB4047C57533CD9D0

#include <string>
#include <vector>
#include <functional>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <cassert>

#define DEG_INV(radian) (radian*180.0)/(M_PI)
#define DEG(angle) ((angle) / 180.0 *M_PI)

namespace dxf {
namespace core {

enum dxf_entity_type_t {
        LINE            = 2023,
        VERTEX          = 2024,
        ARC             = 2025,
        CIRCLE          = 2026,
        ELLIPSE         = 2027,
        FACE            = 2028,
        TEXT            = 2029,
        MTEXT           = 2030,
        POLYLINE        = 2031,
        DIMALIGNED      = 2032,
        SOLID           = 2033,
};

struct point_t {
        double    x, y, z;
        int       color_index;
        double    angle;
public:
        point_t(double vx = 0.0, double vy = 0.0, double vz = 0.0,
                int vcolor_index = 256, double vangle = 0.0) : x(vx), y(vy), z(vz), color_index(vcolor_index), angle(vangle)
        {
                //nothing.
        }
private:
        friend std::istream & operator >> (std::istream & in, point_t & pt);
        friend std::ostream & operator << (std::ostream & out, const point_t & pt);
};

std::istream & operator >> (std::istream & in, point_t & pt)
{
        in >> pt.x >> pt.y >> pt.z;
        pt.color_index = -1;
        pt.angle = 0.0;

        return in;
}

std::ostream & operator << (std::ostream & out, const point_t & pt)
{
        out << pt.x << " " << pt.y << " " << pt.z;
        return out;
}

typedef point_t scale_t;
typedef point_t stretch_direction_t;
typedef point_t vector_t;

struct TBlockNode
{
        std::string         m_strBlockName;
        std::string         m_strBlockID;
        point_t             m_ptBlockBase;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct TInsertNode
{
        std::string	    m_strBlockName;
        std::string	    m_strParentBlockID;
        std::string         m_strLayerName;
        point_t	            m_ptInsertBase;
        scale_t	            m_vecInsertScale;
        double              m_dRotateAngle;
        stretch_direction_t m_vecInsertStretchDirection;
        int                 m_iColorIndex;
};

struct line_t {
        point_t             m_ptStart;
        point_t             m_ptEnd;
        std::string         m_strOwnerID;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct vertex_t {
        point_t             m_ptVertex;
        double              m_bulge;
        std::string         m_strOwnerID;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct polyline_t { 
        std::vector<point_t> m_vecPoints;
        std::string m_strOwnerID; 
        std::string m_strLayerName; 
        int m_iColorIndex;
}; 

struct arc_t {
        point_t             m_ptCenter;
        double              m_dRadius;
        double              m_dStartAngle;
        double              m_dEndAngle;
        std::string         m_strOwnerID;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct circle_t {
        point_t             m_ptCenter;
        double              m_dRadius;
        std::string         m_strOwnerID;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct face_3d_t {
        point_t             m_pt1, m_pt2, m_pt3, m_pt4;
        double              m_dThickness;
        std::string         m_strOwnerID;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct elipse_t {
        point_t             m_ptCenter;
        double              m_dMajorRadius;
        double              m_dMinorRadius;
        vector_t            m_dirMajor;
        vector_t            m_dirMinor;
        double              m_dAngle1;
        double              m_dAngle2;
        std::string         m_strOwnerID;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct text_t {
        point_t             m_ptInsert;
        point_t             m_ptEnd;
        double              m_dHeight;
        double              m_dXScaleFactor;
        int                 m_dTextGenerationFlags;
        int                 m_hJustification;
        int                 m_vJustification;
        std::string         m_strText;
        std::string         m_strStyle;
        double              m_dRotateAngle;
        std::string         m_strOwnerID;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct mtext_t {
        point_t             m_ptInsert;
        double              m_dHeight;
        double              m_dWidth;
        int                 m_iAttachmentPoint;
        int                 m_dDrawingDirection;
        int                 m_dLineSpacingStyle;
        double              m_dLineSpacingFactor;
        std::string         m_strText;
        std::string         m_strStyle;
        double              m_dAngle;
        std::string         m_strOwnerID;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct dimAligned_t {
        point_t             m_pt1;
        point_t             m_pt2;
        point_t             m_pt3;
        point_t             m_ptMText;
        double              m_dAngle;  // Angle in radians
        double              m_dTextRotation;  // Text rotation angle in radians
        int                 m_AttachmentPoint;
        int                 m_dLineSpacingStyle;
        double              m_dLineSpacingFactor;
        std::string         m_strText;
        std::string         m_strStyle;
        std::string         m_strOwnerID;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct solid_t {
        point_t             m_pt1, m_pt2, m_pt3, m_pt4;
        double              m_dThickness;
        std::string         m_strOwnerID;
        std::string         m_strLayerName;
        int                 m_iColorIndex;
};

struct block_t {
        std::string         m_strName;
};

struct layer_t {
        std::string         m_strName;
        int                 m_iColorIndex;
};

std::string Trim(const std::string &input) // Remove leading and trailing whitespace
{
        std::string ret = input;

        static const char whitespace[] = " \n\t\v\r\f";
        ret.erase(0, ret.find_first_not_of(whitespace));
        ret.erase(ret.find_last_not_of(whitespace) + 1U);

        return ret;
}

struct variable_t {
        std::string         m_strVariable;
        int                 m_iCode;
        enum {
                STRING,
                VECTOR,
                INT,
                DOUBLE
        } m_eType;
        template<typename Ret>
        Ret Cast()
        {
                if (std::is_same<Ret, vector_t>::value)
                {
                        assert(m_eType == VECTOR);
                }
                else if (std::is_same<Ret, int>::value)
                {
                        assert(m_eType == INT);
                }
                else if (std::is_same<Ret, double>::value)
                {
                        assert(m_eType == DOUBLE);
                }
                else if (std::is_same<Ret, std::string>::value)
                {
                        //assert(m_eType == STRING);
                        return Trim(m_strVariable);
                }
                else
                {
                        assert(false);
                }

                std::stringstream sstr(m_strVariable.c_str());

                Ret ret;
                sstr >> ret;

                return ret;
        }
};

template<typename T>std::string toString(const T &val)
{
        std::stringstream sstr;
        sstr << val;

        return sstr.str();
}

class TEntityContainer //
{
        std::vector<line_t>     _lines;
        std::vector<vertex_t>   _vertexs;
        std::vector<arc_t>      _arcs;
        std::vector<circle_t>   _circles;
        std::vector<elipse_t>   _elipses;
        std::vector<face_3d_t>  _faces;
        std::vector<polyline_t>  _polylines;
        std::vector<dimAligned_t>  _dimAligneds;
        std::vector<solid_t>  _solids;
        std::vector<text_t>     _texts;
        std::vector<mtext_t>    _mtexts;
        std::vector<block_t>    _blocks;
        std::vector<layer_t>    _layers;
        std::map<std::string, variable_t> _variables;
public:
        TEntityContainer() {}
public:
        void clear()
        {
                _lines.clear();
                _vertexs.clear();
                _arcs.clear();
                _circles.clear();
                _elipses.clear();
                _polylines.clear();
                _dimAligneds.clear();
                _solids.clear();
                _faces.clear();
                _texts.clear();
                _mtexts.clear();
                _blocks.clear();
                _layers.clear();
        }
public:
        void push_back(const line_t &line) {
                _lines.push_back(line);
        }
        void push_back(const polyline_t &polyline) {
                _polylines.push_back(polyline);
        }
        void push_back(const dimAligned_t &dimAligned) {
                _dimAligneds.push_back(dimAligned);
        }
        void push_back(const solid_t &solid) {
                _solids.push_back(solid);
        }
        void push_back(const vertex_t &vertex) {
                _vertexs.push_back(vertex);
        }
        void push_back(const arc_t &arc) {
                _arcs.push_back(arc);
        }
        void push_back(const circle_t &circle) {
                _circles.push_back(circle);
        }
        void push_back(const elipse_t &elipse) {
                _elipses.push_back(elipse);
        }
        void push_back(const face_3d_t &face) {
                _faces.push_back(face);
        }
        void push_back(const text_t &text)
        {
                _texts.push_back(text);
        }
        void push_back(const mtext_t &mtext)
        {
                _mtexts.push_back(mtext);
        }
        void push_back(const block_t &block)
        {
                _blocks.push_back(block);
        }
        void push_back(const layer_t &layer)
        {
                _layers.push_back(layer);
        }
        void insertVariable(const std::string &key, const variable_t &var)
        {
                _variables[key] = var;
        }
        bool isVariableExist(const std::string &key)
        {
                return _variables.find(key) != _variables.end();
        }
        template<typename Ret>
        Ret getVariable(const std::string &key)
        {
                assert(_variables.find(key) != _variables.end());
                return _variables[key].Cast<Ret>();
        }

public:
        template<typename F> void for_each_line(F func) {
                for_each(_lines.begin(), _lines.end(), func);
        }
        template<typename F> void for_each_vertex(F func) {
                for_each(_vertexs.begin(), _vertexs.end(), func);
        }
        template<typename F> void for_each_arc(F func) {
                for_each(_arcs.begin(), _arcs.end(), func);
        }
        template<typename F> void for_each_circle(F func) {
                for_each(_circles.begin(), _circles.end(), func);
        }
        template<typename F> void for_each_elipse(F func) {
                for_each(_elipses.begin(), _elipses.end(), func);
        }
        template<typename F> void for_each_polyline(F func) {
                for_each(_polylines.begin(), _polylines.end(), func);
        }
        template<typename F> void for_each_dimAligned(F func) {
                for_each(_dimAligneds.begin(), _dimAligneds.end(), func);
        }
        template<typename F> void for_each_solid(F func) {
                for_each(_solids.begin(), _solids.end(), func);
        }
        template<typename F> void for_each_face(F func) {
                for_each(_faces.begin(), _faces.end(), func);
        }
        template<typename F> void for_each_text(F func) {
                for_each(_texts.begin(), _texts.end(), func);
        }
        template<typename F> void for_each_mtext(F func) {
                for_each(_mtexts.begin(), _mtexts.end(), func);
        }
        template<typename F> void for_each_block(F func) {
                for_each(_blocks.begin(), _blocks.end(), func);
        }
        template<typename F> void for_each_layer(F func) {
                for_each(_layers.begin(), _layers.end(), func);
        }
public:
        std::string getLayerOfEntity(int mid, int sid)
        {
                if(mid == LINE) return _lines[sid].m_strLayerName;
                if(mid == VERTEX) return _vertexs[sid].m_strLayerName;
                if(mid == ARC) return _arcs[sid].m_strLayerName;
                if(mid == CIRCLE) return _circles[sid].m_strLayerName;
                if(mid == ELLIPSE) return _elipses[sid].m_strLayerName;
                if(mid == POLYLINE) return _polylines[sid].m_strLayerName;
                if(mid == DIMALIGNED) return _dimAligneds[sid].m_strLayerName;
                if(mid == SOLID) return _solids[sid].m_strLayerName; 
                if(mid == FACE) return _faces[sid].m_strLayerName;
                if(mid == TEXT) return _texts[sid].m_strLayerName;
                if(mid == MTEXT) return _mtexts[sid].m_strLayerName;

                return "undefined";
        }

//New Write Begin
        std::string GetEntitiesOfBlock(std::string block_name){
                std::string strRet;
                
                return strRet.substr(0, strRet.length() - 1);
        }
//New Write End
        
};
}
}

#endif // !_DXF_CORE_DATA_16F4EFB686D642BDB4047C57533CD9D0
