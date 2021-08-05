#include <tinyxml2/tinyxml2.h>
// that is only a lexer
//#include <stb/stb_c_lexer.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include <stack>
#include <string>
#include <sstream>
#include <assert.h>
#include "clang-c/Index.h"

typedef unsigned ID;

constexpr ID InvalidID = -1;

template <typename T>
struct Vec2{
	T X, Y;
};

typedef Vec2<int> Vec2s;
typedef Vec2<float> Vec2f;



struct IdGenerator{
	ID BaseId = 1;

	ID NextId(){
		return ++BaseId;
	}
};

std::string ToHexColor(uint32_t color){
	uint8_t red = color >> 24;
	uint8_t green = (color & 0x00FF0000) >> 16;
	uint8_t blue = (color & 0x0000FF00) >> 8;
	uint8_t alpha = color & 0x000000FF;

	char buffer[10];
	sprintf(buffer, "%02x%02x%02x%02x%c", red, green, blue, alpha, '\0');
	return buffer;
}

enum class MxCellType{
	Rect,
	Rhombus,
	Swimlane,
	Text
};

enum class MxConnection{
	Top,
	Bottom,
	Left,
	Right
};

enum class MxArrowStyle{
	None,
	Classic
};

struct MxCellStyle{
	std::string Value = "";
	bool Shadow = false;
	bool Rounded = false;
	uint32_t FillColor = 0xFFFFFFFF;
	uint32_t StrokeColor = 0x000000FF;
};

struct MxGraphBuilder{
	static constexpr ID RootID = 1;

	FILE *OutputFile;
	tinyxml2::XMLPrinter Printer;

	IdGenerator ElementsIdGenerator;

	MxGraphBuilder(const char *filename):
		OutputFile(fopen(filename, "w")),
		Printer(OutputFile)
	{
		Printer.OpenElement("mxGraphModel");
		Printer.OpenElement("root");

		Printer.OpenElement("mxCell");
		Printer.PushAttribute("id", 0);
		Printer.CloseElement();

		Printer.OpenElement("mxCell");
		Printer.PushAttribute("id", 1);
		Printer.PushAttribute("parent", 0);
		Printer.CloseElement();
	}

	~MxGraphBuilder(){
		Printer.CloseElement();//root
		Printer.CloseElement();//mxGraphModel

		fclose(OutputFile);
	}

	ID Cell(MxCellType type, int x, int y, unsigned width, unsigned height, const MxCellStyle &style = {}){
		ID id = ElementsIdGenerator.NextId();
		Printer.OpenElement("mxCell");
		Printer.PushAttribute("id", id);
		Printer.PushAttribute("parent", RootID);
		Printer.PushAttribute("vertex", 1);//who knows what is it
		if(style.Value.size())
			Printer.PushAttribute("value", style.Value.c_str());
		
		static const char *s_Type[]={
			"",
			"rhombus",
			"swimlane",
			"text"
		};
		std::stringstream style_stream;
		style_stream << "whiteSpace=wrap;html=1;";
		if(type != MxCellType::Rect)
			style_stream << s_Type[(size_t)type] << ";";
		style_stream << "shadow=" 		<<(int)style.Shadow 			<< ";";
		style_stream << "glass=" 		<<(int)0						<< ";";
		style_stream << "sketch=" 		<<(int)0						<< ";";
		style_stream << "rounded=" 		<<(int)style.Rounded			<< ";";
		style_stream << "fillColor=#" 	<< ToHexColor(style.FillColor) 	<< ";";
		style_stream << "strokeColor=#" << ToHexColor(style.StrokeColor)<< ";";


		Printer.PushAttribute("style", style_stream.str().c_str());


			Printer.OpenElement("mxGeometry");
			Printer.PushAttribute("x", x);
			Printer.PushAttribute("y", y);
			Printer.PushAttribute("width", width);
			Printer.PushAttribute("height", height);
			Printer.PushAttribute("as", "geometry");
			Printer.CloseElement();
		
		Printer.CloseElement();

		return id;
	}

	void Arrow(ID src, MxConnection src_c, ID dst, MxConnection dst_c, MxArrowStyle style = MxArrowStyle::Classic, const std::vector<Vec2s> &points = {}){

		static Vec2f s_ConnectionPosition[4]={
			{ 0.5f, 0.0f},
			{ 0.5f, 1.0f},
			{ 0.0f, 0.5f},
			{ 1.0f, 0.5f}
		};
		auto src_position = s_ConnectionPosition[(size_t)src_c];
		auto dst_position = s_ConnectionPosition[(size_t)dst_c];

		static const char *s_Arrow[]={
			"none",
			"classic"
		};

		std::stringstream style_stream;
		style_stream << "edgeStyle=orthogonalEdgeStyle;rounded=0;orthogonalLoop=1;jettySize=auto;";
		style_stream << "exitX=" << src_position.X << ";exitY=" << src_position.Y << ";";
		style_stream << "entryX="<< dst_position.X << ";entryY="<< dst_position.Y << ";";
		style_stream << "exitDx=0;exitDy=0;entryDx=0;entryDy=0;";
		style_stream << "endArrow=" << s_Arrow[(size_t)style];


		Printer.OpenElement("mxCell");
		Printer.PushAttribute("id", ElementsIdGenerator.NextId());
		Printer.PushAttribute("parent", RootID);
		Printer.PushAttribute("vertex", 1);
		Printer.PushAttribute("source", src);
		Printer.PushAttribute("target", dst);
		Printer.PushAttribute("edge", 1);
		Printer.PushAttribute("style", style_stream.str().c_str());
			Printer.OpenElement("mxGeometry");
			Printer.PushAttribute("relative", 1);
			Printer.PushAttribute("as", "geometry");

			if(points.size()){
				Printer.OpenElement("Array");
				Printer.PushAttribute("as", "points");
				for(const auto &point: points){
					Printer.OpenElement("mxPoint");
					Printer.PushAttribute("x", point.X);
					Printer.PushAttribute("y", point.Y);
					Printer.CloseElement();
				}
				Printer.CloseElement();
			}
			Printer.CloseElement();
		Printer.CloseElement();
	}
};

struct InsertedCellInfo{
	ID Id;
	MxConnection SrcConnectionSide;

	InsertedCellInfo(){
		Reset();
	}

	void Next(ID id, MxConnection src_connection){
		Id = id;
		SrcConnectionSide = src_connection;
	}

	void Reset(){
		Id = InvalidID;
		SrcConnectionSide = MxConnection::Bottom;
	}
};

struct ScopeSize{
	int Left = 0;
	int Right= 0;
};

ScopeSize GetScopeSize(const std::vector<ScopeSize> &scope){
	ScopeSize scope_size;
	for(auto size: scope){
		if(size.Left > scope_size.Left)
			scope_size.Left = size.Left;
		if(size.Right > scope_size.Right)
			scope_size.Right = size.Right;
	}
	return scope_size;
}

template<typename T>
class ScopeStack{
private:
	std::stack<std::vector<T>> m_Impl;
public:
	void Clear(){
		std::stack<std::vector<T>>().swap(m_Impl);
	}

	void Push(const T &value){
		m_Impl.top().push_back(value);
	}

	void BeginScope(){
		m_Impl.push({});
	}

	std::vector<T> EndScope(){
		std::vector<T> scope = std::move(m_Impl.top());
		m_Impl.pop();
		return scope;
	}
};

struct ControlFlowDiagramBuilder{
	const int FunctionSizeX = 200;

	static constexpr int BlockHeight = 30;
	static constexpr int CellSpacing = 20;
	static constexpr int RhombusSizeX = BlockHeight * 2;
	static constexpr int RhombusSizeY = BlockHeight;

	static constexpr int IndentSize = 8;
	static constexpr int FunctionOffset = 15;


	MxGraphBuilder DiagramBuilder;

	int CursorX = 0;
	int CursorY;

	//@WidthStack
	ScopeStack<ScopeSize> ScopeWidthStack;

	int LeftIndentationSize  = 0;
	int RightIndentationSize = 0;

	InsertedCellInfo LastCellInfo;

	bool IsInFunction;


	ControlFlowDiagramBuilder(const char *filename):
		DiagramBuilder(filename)
	{
		ResetStateForNextFunction();
	}

	ID Cell(MxCellType type, int width, int height, const MxCellStyle &style, MxConnection connection_type = MxConnection::Bottom){
		ID id = DiagramBuilder.Cell(type, CursorX - width/2, CursorY, width, height, style);

		if(LastCellInfo.Id != InvalidID)
			Arrow(id, MxConnection::Top);

		CursorY += height + CellSpacing;

		LastCellInfo.Next(id, connection_type);

		ScopeWidthStack.Push({width/2, width/2});

		return id;
	}

	void Arrow(ID dst, MxConnection dst_c, MxArrowStyle style = MxArrowStyle::Classic, const std::vector<Vec2s> &points = {}){
		DiagramBuilder.Arrow(LastCellInfo.Id, LastCellInfo.SrcConnectionSide, dst, dst_c, style, points);
	}

	ID BindPoint(){
		return DiagramBuilder.Cell(MxCellType::Rect, CursorX, CursorY, 1, 1);
	}

	void PushBlock(const MxCellStyle &style){
		Cell(MxCellType::Rect, FunctionSizeX, BlockHeight, style);
	}

	void ResetStateForNextFunction(){
		CursorY = 0;

		LastCellInfo.Reset();

		ScopeWidthStack.Clear();

		IsInFunction = false;
	}

	void PushFunction(const char *declaration){
		if(IsInFunction){
			ScopeSize funcion_body_scope_size = GetScopeSize(ScopeWidthStack.EndScope());

			CursorX += funcion_body_scope_size.Left + funcion_body_scope_size.Right + FunctionOffset;
			ResetStateForNextFunction();
		}
		IsInFunction = true;

		ScopeWidthStack.BeginScope();

		Cell(MxCellType::Rect, FunctionSizeX, BlockHeight, {declaration, false, true});
	}

	void PushStatement(const char *name){
		Cell(MxCellType::Rect, FunctionSizeX, BlockHeight, {name, false, false});
	}

	void PushFunctionCall(const char *name){
		Cell(MxCellType::Swimlane, FunctionSizeX, BlockHeight * 2, {name, true, false, 0x9922FFFF});
	}

};

CXCursor clang_getCursorInChildByIndex(CXCursor cursor, size_t index){
	struct CountingCursor{
		CXCursor Cursor;
		size_t Counter;
		size_t DesiredIndex;
	}counting_cursor{clang_getNullCursor(), 0, size_t(-1)};

	counting_cursor.DesiredIndex = index;

	clang_visitChildren(cursor, [](CXCursor cursor, CXCursor parent, CXClientData clientData){
		CountingCursor &counting_cursor = *static_cast<CountingCursor*>(clientData);

		if(counting_cursor.Counter++ == counting_cursor.DesiredIndex){
			counting_cursor.Cursor = cursor;
			return CXChildVisit_Break;
		}

		return CXChildVisit_Continue;
	}, &counting_cursor);

	return counting_cursor.Cursor;
}

ControlFlowDiagramBuilder &GetBuilder(CXClientData data){
	return *static_cast<ControlFlowDiagramBuilder*>(data);
}

CXChildVisitResult ASTScopeTraverser(CXCursor cursor, CXCursor parent, CXClientData clientData);

void TraverseSubtree(CXCursor cursor, CXClientData clientData){
	clang_visitChildren(cursor, ASTScopeTraverser, clientData);
}

struct EntryControlledLoopStmtBuilder{
	ControlFlowDiagramBuilder &Builder;
	ID ConditionCell;
	int StatementBeginCursorY;
	int StatementEndCursorY;
	const char *UpdateStatement;

	EntryControlledLoopStmtBuilder(ControlFlowDiagramBuilder &builder, const char *condition = nullptr, const char *init = nullptr, const char *update = nullptr):
		Builder(builder),
		UpdateStatement(update)
	{
		Builder.ScopeWidthStack.BeginScope();

		if(init && std::strlen(init))
			Builder.PushStatement(init);
		
		ConditionCell = Builder.Cell(MxCellType::Rhombus, ControlFlowDiagramBuilder::RhombusSizeX, ControlFlowDiagramBuilder::RhombusSizeY, {condition?condition:""});

		StatementBeginCursorY = Builder.CursorY;
	}

	~EntryControlledLoopStmtBuilder(){

		if(UpdateStatement && std::strlen(UpdateStatement))
			Builder.PushStatement(UpdateStatement);

		std::vector<ScopeSize> scope = Builder.ScopeWidthStack.EndScope();

		ScopeSize loop_body_scope_size = GetScopeSize(scope);

		Builder.ScopeWidthStack.Push({loop_body_scope_size.Left + ControlFlowDiagramBuilder::IndentSize, loop_body_scope_size.Right + ControlFlowDiagramBuilder::IndentSize});


		Builder.CursorY += Builder.CellSpacing/2;
		StatementEndCursorY = Builder.CursorY;

		// XXX why am i using ints here?

	
		Builder.Arrow(ConditionCell, MxConnection::Right, MxArrowStyle::Classic,{
			{Builder.CursorX, StatementEndCursorY - Builder.CellSpacing},

			{Builder.CursorX + loop_body_scope_size.Right + ControlFlowDiagramBuilder::IndentSize, StatementEndCursorY  - Builder.CellSpacing},
			{Builder.CursorX + loop_body_scope_size.Right + ControlFlowDiagramBuilder::IndentSize, StatementBeginCursorY- ControlFlowDiagramBuilder::RhombusSizeY/2 - Builder.CellSpacing},

			{Builder.CursorX + ControlFlowDiagramBuilder::RhombusSizeX/2, StatementBeginCursorY- ControlFlowDiagramBuilder::RhombusSizeY/2 - Builder.CellSpacing},
		});

		ID point = Builder.BindPoint();
		Builder.LastCellInfo.Next(point, MxConnection::Bottom);
		Builder.CursorY += Builder.CellSpacing/2;

		Builder.DiagramBuilder.Arrow(ConditionCell, MxConnection::Left, point, MxConnection::Top, MxArrowStyle::None, {
			{Builder.CursorX - ControlFlowDiagramBuilder::RhombusSizeX/2, StatementBeginCursorY - ControlFlowDiagramBuilder::RhombusSizeY/2 - Builder.CellSpacing},
			{Builder.CursorX - (loop_body_scope_size.Left + ControlFlowDiagramBuilder::IndentSize), StatementBeginCursorY- ControlFlowDiagramBuilder::RhombusSizeY/2 - Builder.CellSpacing},
			{Builder.CursorX - (loop_body_scope_size.Left + ControlFlowDiagramBuilder::IndentSize), StatementEndCursorY  - Builder.CellSpacing/2},
			{Builder.CursorX, StatementEndCursorY - Builder.CellSpacing/2}
		});
	}

};

void TraverseForLoopStatement(CXCursor body, CXCursor initialization, CXCursor condition, CXCursor update, CXClientData clientData){
	ControlFlowDiagramBuilder &builder = GetBuilder(clientData);

	EntryControlledLoopStmtBuilder stmt(builder, 
		// if cursor is nullCursor, nullptr is returned
		clang_getCString(clang_getCursorDisplayName(condition)),
		clang_getCString(clang_getCursorDisplayName(initialization)),
		clang_getCString(clang_getCursorDisplayName(update))
	);

	TraverseSubtree(body, clientData);
}
void TraverseWhileLoopStatement(CXCursor body, CXCursor condition, CXClientData clientData){
	auto null = clang_getNullCursor();
	TraverseForLoopStatement(body, null, condition, null, clientData);
}

struct ExitControlledLoopStmtBuilder{
	ControlFlowDiagramBuilder &Builder;
	const char *UpdateStatement;
	int StatementBeginCursorY;
	int StatementEndCursorY;

	ID LoopBeginCell     = InvalidID;
	ID LoopConditionCell = InvalidID;

	ExitControlledLoopStmtBuilder(ControlFlowDiagramBuilder &builder, const char *update = nullptr):
		Builder(builder),
		UpdateStatement(update)
	{
		StatementBeginCursorY = Builder.CursorY;
		LoopBeginCell = Builder.BindPoint();
		Builder.CursorY += Builder.CellSpacing;

		Builder.ScopeWidthStack.BeginScope();
	}


	~ExitControlledLoopStmtBuilder(){
		
		std::vector<ScopeSize> scope = Builder.ScopeWidthStack.EndScope();

		ScopeSize loop_body_scope_size = GetScopeSize(scope);

		Builder.ScopeWidthStack.Push({loop_body_scope_size.Left + ControlFlowDiagramBuilder::IndentSize, loop_body_scope_size.Right + ControlFlowDiagramBuilder::IndentSize});
		
		// here we can handle less state
		// P.S. Who cares?
		StatementEndCursorY = Builder.CursorY;
		LoopConditionCell = Builder.Cell(MxCellType::Rhombus, ControlFlowDiagramBuilder::RhombusSizeX, ControlFlowDiagramBuilder::RhombusSizeY, {UpdateStatement?UpdateStatement:""}, MxConnection::Bottom);			

		std::vector<Vec2s> array = {
			{Builder.CursorX - (loop_body_scope_size.Left + ControlFlowDiagramBuilder::IndentSize), StatementEndCursorY + ControlFlowDiagramBuilder::RhombusSizeY/2},
			{Builder.CursorX - (loop_body_scope_size.Left + ControlFlowDiagramBuilder::IndentSize), StatementBeginCursorY}
		};
		Builder.DiagramBuilder.Arrow(LoopConditionCell, MxConnection::Left, LoopBeginCell, MxConnection::Left, MxArrowStyle::Classic, array);

	}

};

void TraverseDoStatement(CXCursor body, CXCursor condition, CXClientData clientData){
	ControlFlowDiagramBuilder &builder = GetBuilder(clientData);
	ExitControlledLoopStmtBuilder stmt(builder, clang_getCString(clang_getCursorDisplayName(condition)));

	TraverseSubtree(body, clientData);
}

struct IfStmtBuilder{
	ControlFlowDiagramBuilder &Builder;
	int StatementBeginCursorY;
	int IfEndCursorY;
	int ElseEndCursorY;
	ID ConditionCell;

	InsertedCellInfo LastIfCell;
	InsertedCellInfo LastElseCell;

	ScopeSize IfScopeSize;
	ScopeSize ElseScopeSize;

	IfStmtBuilder(ControlFlowDiagramBuilder &builder, const char *condition = nullptr):
		Builder(builder)
	{
		ConditionCell = builder.Cell(MxCellType::Rhombus, ControlFlowDiagramBuilder::RhombusSizeX, ControlFlowDiagramBuilder::RhombusSizeY, {condition?condition:""}, MxConnection::Right);
		StatementBeginCursorY = builder.CursorY;
	}

	void BeginIfBody(){
		Builder.LastCellInfo.Next(ConditionCell, MxConnection::Bottom);

		Builder.ScopeWidthStack.BeginScope();

		Builder.ScopeWidthStack.Push({0, Builder.FunctionSizeX/2});

		Builder.CursorY = StatementBeginCursorY;
	}

	void EndIfBody(){
		IfEndCursorY = Builder.CursorY;

		std::vector<ScopeSize> scope = Builder.ScopeWidthStack.EndScope();
		IfScopeSize = GetScopeSize(scope);

		LastIfCell = Builder.LastCellInfo;
	}

	void BeginElseBody(){
		Builder.LastCellInfo.Next(ConditionCell, MxConnection::Right);

		Builder.ScopeWidthStack.BeginScope();

		Builder.CursorY = StatementBeginCursorY;
		Builder.CursorX += IfScopeSize.Right + IfScopeSize.Left + ControlFlowDiagramBuilder::IndentSize;
	}

	void EndElseBody(){
		Builder.CursorX -= IfScopeSize.Right + IfScopeSize.Left + ControlFlowDiagramBuilder::IndentSize;

		ElseEndCursorY = Builder.CursorY;

		std::vector<ScopeSize> scope = Builder.ScopeWidthStack.EndScope();
		ElseScopeSize = GetScopeSize(scope);

		LastElseCell = Builder.LastCellInfo;
	}

	~IfStmtBuilder(){
		assert(LastIfCell.Id   != InvalidID);
		assert(LastElseCell.Id != InvalidID);

		Builder.CursorY = std::max(IfEndCursorY, ElseEndCursorY);

		Builder.ScopeWidthStack.Push({IfScopeSize.Left, IfScopeSize.Right + ControlFlowDiagramBuilder::IndentSize + ElseScopeSize.Left + ElseScopeSize.Right});

		ID point = Builder.BindPoint();


		Builder.LastCellInfo = LastIfCell;
		Builder.Arrow(point, MxConnection::Top, MxArrowStyle::None);

		std::vector<Vec2s> array;
		if(LastElseCell.Id == ConditionCell)
			array.push_back({Builder.CursorX + IfScopeSize.Right + ControlFlowDiagramBuilder::IndentSize, StatementBeginCursorY});
		array.push_back(    {Builder.CursorX + IfScopeSize.Right + ControlFlowDiagramBuilder::IndentSize, Builder.CursorY});
		array.push_back(    {Builder.CursorX + ControlFlowDiagramBuilder::IndentSize,                     Builder.CursorY});
		Builder.DiagramBuilder.Arrow(LastElseCell.Id, LastElseCell.SrcConnectionSide, point, MxConnection::Right, MxArrowStyle::Classic, array);

		Builder.CursorY += Builder.CellSpacing;

		Builder.LastCellInfo.Next(point, MxConnection::Bottom);
	}
};

void TraverseIfStatement(CXCursor condition, CXCursor if_body, CXCursor else_body, CXClientData clientData){

	ControlFlowDiagramBuilder &builder = GetBuilder(clientData);

	IfStmtBuilder stmt(builder, "well, if...");

	stmt.BeginIfBody();
	{
		ASTScopeTraverser(if_body, clang_getCursorSemanticParent(if_body), clientData);
	}
	stmt.EndIfBody();

	stmt.BeginElseBody();
	{
		ASTScopeTraverser(else_body, clang_getCursorSemanticParent(else_body), clientData);
	}
	stmt.EndElseBody();

}

CXChildVisitResult ASTScopeTraverser( CXCursor cursor, CXCursor parent , CXClientData clientData){
	ControlFlowDiagramBuilder &builder = GetBuilder(clientData);

	CXCursorKind cursorKind = clang_getCursorKind( cursor );
	
	switch(cursorKind){
	case CXCursor_CompoundStmt:
		clang_visitChildren( cursor, ASTScopeTraverser, clientData);
		break;
	case CXCursor_DeclStmt:
		builder.PushBlock({"Delcare variables", false, false, 0x2299FFFF});
		break;
	//case CXCursor_VarDecl:
	case CXCursor_DeclRefExpr:
		builder.PushStatement(clang_getCString(clang_getCursorDisplayName(cursor)));
		break;
	case CXCursor_CallExpr:
		builder.PushFunctionCall(clang_getCString(clang_getCursorDisplayName(cursor)));
		break;
	case CXCursor_CaseStmt:
		break;
	case CXCursor_DefaultStmt:
		break;
	case CXCursor_IfStmt:{
		CXCursor condition = clang_getCursorInChildByIndex(cursor, 0);
		CXCursor if_body   = clang_getCursorInChildByIndex(cursor, 1);
		CXCursor else_body = clang_getCursorInChildByIndex(cursor, 2);
		TraverseIfStatement(condition, if_body, else_body, clientData);
	}break;
	case CXCursor_SwitchStmt:
		break;
	case CXCursor_WhileStmt:{
		CXCursor condition = clang_getCursorInChildByIndex(cursor, 0);
		CXCursor body      = clang_getCursorInChildByIndex(cursor, 1);
		TraverseWhileLoopStatement(body, condition, clientData);
	}break;
	case CXCursor_DoStmt:{
		CXCursor body      = clang_getCursorInChildByIndex(cursor, 0);
		CXCursor condition = clang_getCursorInChildByIndex(cursor, 1);
		TraverseDoStatement(body, condition, clientData);
	}break;
	case CXCursor_ForStmt:{
		CXCursor initialization = clang_getCursorInChildByIndex(cursor, 0);
		CXCursor condition      = clang_getCursorInChildByIndex(cursor, 1);
		CXCursor update         = clang_getCursorInChildByIndex(cursor, 2);
		CXCursor body           = clang_getCursorInChildByIndex(cursor, 3);
		TraverseForLoopStatement(body, initialization, condition, update, clientData);
	}break;
	case CXCursor_GotoStmt:
		break;
	case CXCursor_IndirectGotoStmt:
		break;
	case CXCursor_ContinueStmt:
		break;
	case CXCursor_BreakStmt:
		break;
	case CXCursor_ReturnStmt:
		builder.PushStatement("Return");
		break;
	default:
		break;
	}

    return CXChildVisit_Continue;
}

CXChildVisitResult ASTFunctionTraverser( CXCursor cursor, CXCursor parent, CXClientData clientData){
    CXCursorKind cursorKind = clang_getCursorKind( cursor );

	if(cursorKind == CXCursor_CompoundStmt){
    	TraverseSubtree(cursor, clientData);
	}

    return CXChildVisit_Continue;
}


bool clang_isFunctionDefinition(CXCursor cursor){
	auto traverse = [](CXCursor cursor, CXCursor, CXClientData){
		CXCursorKind cursorKind = clang_getCursorKind(cursor);

		if(cursorKind == CXCursor_ParmDecl || clang_isAttribute(cursorKind) || cursorKind == CXCursor_TypeRef)
			return CXChildVisit_Continue;
		return CXChildVisit_Break;
	};
	return clang_visitChildren(cursor, traverse, nullptr);
}



CXChildVisitResult ASTTraverser( CXCursor cursor, CXCursor /* parent */, CXClientData clientData){
    CXCursorKind cursorKind = clang_getCursorKind( cursor );
    
	if(cursorKind == CXCursor_FunctionDecl){
		if(clang_isFunctionDefinition(cursor)){
			auto &builder = GetBuilder(clientData);
			builder.PushFunction(clang_getCString(clang_getCursorDisplayName(cursor)));

			clang_visitChildren( cursor, ASTFunctionTraverser, clientData);


			builder.Cell(MxCellType::Rect, builder.FunctionSizeX, builder.BlockHeight, {"End", false, true});
		}
	}

    return CXChildVisit_Continue;
}


int main( int argc, char** argv ){

	CXIndex index = clang_createIndex(0, 1);

	CXTranslationUnit unit = clang_parseTranslationUnit (
        index, // CIdx
        "kp6.c", // source_filename
        argv, // command_line_args
        argc, // num_command_line_args
        0, // unsave_files
        0, // num_unsaved_files
        CXTranslationUnit_None // options
    );
	assert(unit);

	ControlFlowDiagramBuilder builder("graph.xml");


	CXCursor root = clang_getTranslationUnitCursor(unit);
	clang_visitChildren(root, ASTTraverser, &builder);

	return 0;
}