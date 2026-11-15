// CamTabCommand.h : the two-way toggle command that owns the CAM tab.
//
// There is one of these per assembly workspace. It holds that assembly's job,
// and loads and saves its .cam file. While the tab has focus it runs a
// CCamViewCommand, which draws the origins and receives picks.

#pragma once

#include <map>
#include <set>

#include "CommandBase.h"
#include "CamJob.h"
#include "FixtureCheck.h"
#include "CamViewCommand.h"
#include "GeometryReader.h"
#include "Machine.h"
#include "Materials.h"
#include "OperationDialog.h"
#include "OriginDialog.h"
#include "ProgramSettings.h"
#include "ToolLibrary.h"

class CAddOnInterface;
class CCamTabWnd;
class CPartBoxStockDlg;
class CFixturesDlg;
class CSimulationDlg;

// A place in the sequence: a top-level step, or an operation in a group.
struct StepRef
{
    static constexpr size_t kNone = camcore::kNoStepIndex;

    size_t step = kNone;        // index in the sequence
    size_t child = kNone;       // index in that group, for an operation

    bool IsOperation() const { return child != kNone; }

    bool operator==(const StepRef& other) const { return step == other.step && child == other.child; }
    bool operator!=(const StepRef& other) const { return !(*this == other); }
};

// Where a dragged step lands, next to the step it's dropped on. Into is for
// dropping an operation on a folder, which adds it at the folder's end.
enum class DropSide
{
    Before,
    After,
    Into
};

class GenerateProgress;

class CCamTabCommand : public CCommandBase
{
public:
    CCamTabCommand(CAddOnInterface* pAddOn, const std::wstring& sessionId);
    virtual ~CCamTabCommand();

    // Called by the tab window once it's on screen, so a load can show a prompt.
    void LoadJobFile();
    void SaveJobFile();
    void LoadOtherJobFile();        // the Load button: chooses a .cam file
    void SaveJobTemplate();
    void LoadJobTemplate();

    // The machine, its tools, and the job's material. tab is a CMachineDlg::Tab.
    void OpenMachineDialog(int tab);
    void SelectToolByIndex(size_t index);
    const camcore::MaterialList& GetMaterials() const { return m_materials; }
    const camcore::Machine& GetMachine() const { return m_machine; }
    const std::string& GetMaterialId() const { return m_job.materialId; }
    std::wstring GetMaterialName() const;       // "none" when the job has no material
    void ChooseMaterial(const std::string& id);

    // Writes the speeds and feeds to the tool in the machine's file.
    bool SaveToolFeeds(const std::string& toolId, const std::string& materialId, const camcore::FeedSettings& feeds,
                       std::wstring& problem);

    // The stock. There's at most one.
    bool HasStock() const { return m_job.stock.kind != camcore::StockKind::None; }
    void MakeStock(camcore::StockKind kind);
    void EditStock();
    void DeleteStock();
    void SetStockShown(bool shown);
    void SetOriginShown(bool shown);

    // From the Part Box Stock dialog
    void BeginPartSelect();
    void EndPartSelect();
    void OnStockPreviewChanged();
    void OnPartBoxDialogDone(CPartBoxStockDlg* pDialog, bool apply);

    // From the Fixtures dialog
    void OnFixturesPreviewChanged();
    void OnFixturesDialogDone(CFixturesDlg* pDialog, bool apply);

    // The operation sequence
    void AddSetOrigin();
    void AddFixtures();
    void AddFolder();
    void AddOperation(size_t folder, camcore::StepType type);
    bool CanAddFolder() const;
    bool IsFolder(const StepRef& ref) const;
    bool IsOperation(const StepRef& ref) const;
    bool CanRename(const StepRef& ref) const;
    bool CanDelete(const StepRef& ref) const;
    bool RenameStep(const StepRef& ref, const std::wstring& name);      // false keeps the old name
    void EditStep(const StepRef& ref);
    void DeleteStep(const StepRef& ref);
    void SelectStep(const StepRef& ref);
    void DuplicateStep(const StepRef& ref);     // the copy goes just below
    bool IsSuppressed(const StepRef& ref) const;
    bool CanSuppress(const StepRef& ref) const;
    void ToggleSuppressed(const StepRef& ref);

    // Calculates toolpaths: every operation with no step, a folder's, or one operation's.
    bool CanGenerate() const;
    void Generate(const StepRef& scope);

    // Every operation in scope, suppressed or not: all with no step, a folder's, or one.
    std::vector<StepRef> AllOperationsIn(const StepRef& scope) const;

    // Throws away generated toolpaths (and Facing's boundaries). True if any of the operations has one.
    bool HasToolpaths(const std::vector<StepRef>& refs) const;
    void ClearToolpaths(const std::vector<StepRef>& refs);

    // Plays the generated toolpaths of the operations in scope (or those chosen)
    // against the stock, in the Simulation dialog.
    void Simulate(const StepRef& scope);
    void SimulateOperations(std::vector<StepRef> refs);
    camcore::Simulation& GetSimulation() { return m_simulation; }
    // The Resolution slider's stops: cell sizes, or dexel spacings.
    std::vector<double> SimulationStops(camcore::StockModel model) const
    {
        return (model == camcore::StockModel::TriDexel) ? camcore::DexelResolutionStops(m_simulationBox)
                                                        : camcore::ResolutionStops(m_simulationBox);
    }
    void RestartSimulation(const camcore::SimResolution& resolution);   // and cut back to the same move
    std::wstring SimulationGridText() const;                            // the stock's grid, for the log
    std::filesystem::path SimulationStlPath() const;                    // the suggested name, beside the .cam file

    // What the simulation shows: the cut stock, the parts' cells, and the
    // differences from the parts. The parts are read the first time they're needed.
    struct SimulationShow
    {
        bool stock = true;
        bool part = false;
        bool notEnough = false;
        bool tooMuch = false;
        bool fixtures = true;   // the current operation's fixtures, shaded over the stock
    };
    void SetSimulationShow(const SimulationShow& show);
    std::wstring SimulationCompareText() const;     // the volumes, or why the parts can't be compared
    bool SaveSimulationStl(const std::filesystem::path& path, std::wstring& message);
    void OnSimulationChanged();             // the stock or tool moved: mesh again and redraw
    void OnSimulationDialogDone(CSimulationDlg* pDialog);

    // Several operations chosen in the tree with Ctrl or Shift: drawn as chosen,
    // edited together, or generated together.
    void SetChosenOperations(const std::vector<StepRef>& refs);
    void EditOperations(const std::vector<StepRef>& refs);
    void GenerateOperations(const std::vector<StepRef>& refs);

    // Writes the generated toolpaths of every operation, or a folder's, to a file.
    void Post(const StepRef& scope);

    // Dragging to reorder. Steps never change level, and the first step stays a Set Origin.
    bool CanDrag(const StepRef& ref) const;
    bool CanMoveStep(const StepRef& from, const StepRef& target, DropSide side) const;
    void MoveStep(const StepRef& from, const StepRef& target, DropSide side);
    bool CanEditSequence() const;

    // The view command, which draws while the tab has focus
    void OnTabVisibilityChanged(bool visible);
    void RefreshView();
    void SetForeground(bool foreground);
    void SetToolpathsShown(bool shown);
    const CamScene& GetScene() const { return m_scene; }
    void HandleSelection();
    void ClearPickedSelection();
    void OnViewTerminated(CCamViewCommand* pView);

    // From the Set Origin dialog
    void BeginPick(PickSlot slot);
    void EndPick();
    void OnOriginPreviewChanged();
    void OnOriginDialogDone(COriginDlg* pDialog, bool apply);

    // From the operation dialog
    void BeginLocate();
    void EndLocate();
    void BeginFeatureSelect();
    void EndFeatureSelect();
    bool GetOperationFrame(camcore::Frame& frame) const;
    const camcore::ToolLibrary& GetToolLibrary() const { return m_library; }

    // Shows the rows chosen in the Features list in the view.
    void ShowFeaturesInView(const std::vector<camcore::PickedGeometry>& features);

    // The features whose outlines are drawn in orange: the open operation
    // dialog's, otherwise the tree's selected operation's (all of a folder's
    // operations' for a folder). Their outlines are read from the model and kept
    // until the features change.
    std::vector<camcore::PickedGeometry> HighlightedFeatures() const;
    void UpdateHighlights();
    void OnOperationDialogDone(COperationDlg* pDialog, bool apply);
    void OnOperationDialogGenerate(COperationDlg* pDialog);

    void OnWindowClosing();
    void OnWindowGone();
    void CloseFromAddOn();

    // IAlibreAddOnCommand
    STDMETHOD(IsTwoWayToggle)(VARIANT_BOOL* pValue) override;
    STDMETHOD(AddTab)(VARIANT_BOOL* pValue) override;
    STDMETHOD(get_TabName)(BSTR* pName) override;
    STDMETHOD(OnShowUI)(__int64 hWnd) override;
    STDMETHOD(OnTerminate)() override;

private:
    IADSessionPtr GetSession() const;
    void SetStatus(const std::wstring& text);
    void ShowStatus();
    void SetSaveBlocked(bool blocked);
    void WarnIfUnsaved();
    void MarkJobChanged();
    void ReloadTools();         // the job's machine, its tools, and the materials

    // An operation with a toolpath, ready for the simulation. False if it can't run.
    bool MakeSimOperation(const StepRef& ref, camcore::SimOperation& operation) const;
    // Rest Machining: cuts the stock with the operations before this one, and what's
    // left along the frame's Z.
    // Thread Milling: warns of each thread whose hole the operations before it
    // haven't opened to the minor diameter down to the thread's bottom.
    void CheckThreadHoles(const StepRef& target, const camcore::Frame& frame);
    bool ComputeRemainingStock(const StepRef& target, const camcore::Frame& frame, double toolRadius,
                               camcore::RemainingStock& remaining, std::string& problem);
    void UpdateTreeWarnings();  // the red exclamation marks on nodes with a problem
    void ReadJobFile();
    void ShowSequence();
    void DestroyWindowIfOpen();
    void LeaveAddOn();

    bool StartView(const wchar_t* reason);
    void StopView(const wchar_t* reason);
    void ReadModelSize();
    void RebuildScene();
    camcore::SequenceStep* FindStep(const StepRef& ref);
    const camcore::SequenceStep* FindStep(const StepRef& ref) const;
    bool IsDialogOpen() const;
    bool FocusOpenDialog();         // true if a dialog was open
    void OpenOriginDialog(const camcore::SequenceStep& step, size_t editIndex);
    void CloseOriginDialog(bool callAlibre);
    void OpenOperationDialog(const camcore::SequenceStep& step, const StepRef& target);
    void CloseOperationDialog();
    bool PrepareSelection(const SelectionFilter& wanted);     // false if the view couldn't start
    bool IsPicking() const;
    bool GenerateOperation(const StepRef& target, std::string& problem);
    bool GenerateOperationMoves(const StepRef& target, std::string& problem);   // before arcs are fitted
    void GenerateTargets(const std::vector<StepRef>& targets);
    bool IsChosen(size_t step, size_t child) const;   // in the tree's chosen set, or the open dialog's
    bool ApplyOperationStep(camcore::SequenceStep step);
    // The operations in scope. Suppressed ones are left out, unless the scope is
    // the operation itself and it isn't for a post.
    std::vector<StepRef> OperationsIn(const StepRef& scope, bool forPost) const;
    void RestoreSelectionFilter();
    void HandleLocate();
    void OpenStockDialog(camcore::StockKind kind);
    void ApplyStock(const camcore::Stock& stock);
    void ClosePartBoxDialog(bool callAlibre);
    void OpenFixturesDialog(const camcore::SequenceStep& step, size_t editIndex);
    void CloseFixturesDialog(bool callAlibre);

    // The occurrence keys of every fixture part in the job, which aren't part of "the part".
    OccurrenceKeys FixtureKeys() const;

    // The fixtures the operations in a top-level step run among, read once per
    // Generate (or Simulate): none if there's no Fixtures step above it.
    struct FixtureRun
    {
        bool read = false;
        std::wstring problem;                       // why they couldn't be read
        const camcore::FixtureSettings* settings = nullptr;
        std::vector<camcore::FixtureMesh> meshes;   // in assembly mm
        camcore::FixtureGrids grids;
    };
    const FixtureRun& FixturesFor(size_t step);
    void CloseSimulationDialog();
    void UpdateSimulationScene();
    bool PrepareSimulationPart();           // the parts read and put in the simulation's grid
    void ClearSimulationCompare();          // at the start or end of a simulation, or a new grid
    // The stock's edges in assembly mm, and its box. False with a problem if it can't be read.
    // pPoints, if given, gets points whose highest is the stock's top along any direction.
    bool ReadStockShape(const camcore::Stock& stock, std::vector<camcore::LineSegment>& edges,
                        camcore::StockBox& box, std::wstring& problem,
                        std::vector<camcore::Vec3>* pPoints = nullptr);

    CAddOnInterface* m_pAddOn;
    std::wstring m_sessionId;
    CCamTabWnd* m_pWnd;

    camcore::Job m_job;
    std::filesystem::path m_camPath;
    bool m_camFileChosen = false;   // Load chose m_camPath, so Save goes there, not beside the assembly

    // Set when a file exists that saving would wrongly overwrite: one from a
    // newer build, a damaged one, or one for a different assembly.
    bool m_saveBlocked;

    bool m_unsavedChanges;

    camcore::Machine m_machine;         // the job's machine, empty if it can't be read
    camcore::ToolLibrary m_library;     // its tools
    camcore::MaterialList m_materials;
    std::wstring m_jobStatus;
    std::wstring m_toolStatus;
    std::wstring m_sequenceStatus;

    // The selected step. The view draws the origin it runs under. Not saved.
    StepRef m_selected;

    CCamViewCommand* m_pView;           // holds a reference while the view is active
    CamScene m_scene;
    bool m_haveBounds;
    camcore::Vec3 m_boundsLower;
    camcore::Vec3 m_boundsUpper;

    COriginDlg* m_pOriginDlg;
    size_t m_editIndex;                 // the step the dialog edits, or kNewStep

    // Only one dialog is open at a time, so this and m_pOriginDlg aren't both set.
    COperationDlg* m_pOperationDlg;
    StepRef m_operationTarget;          // the folder, and the operation in it or kNewStep
    std::vector<StepRef> m_bulkTargets; // the operations a bulk dialog edits, with m_operationTarget the first
    std::vector<StepRef> m_chosen;      // the operations chosen together in the tree
    bool m_showingFeatures;             // Features list rows are selected in the view
    std::vector<camcore::PickedGeometry> m_highlighted;    // the features m_highlightOutlines were read for
    std::vector<std::vector<camcore::Vec3>> m_highlightOutlines;   // in assembly mm

    // Only one dialog is open at a time, so this is set only when the others aren't.
    CPartBoxStockDlg* m_pPartBoxDlg;
    CFixturesDlg* m_pFixturesDlg = nullptr;
    size_t m_fixturesEditIndex = static_cast<size_t>(-1);  // the step the Fixtures dialog edits, or kNewStep

    // Fixtures, read for the run in progress, by Fixtures step index; and what
    // Generate found: operations (by ID) with a warning (too close to a fixture, a
    // thread's hole not open), Fixtures steps (by ID) whose parts can't be read,
    // and the warnings to show.
    std::map<size_t, FixtureRun> m_fixtureRuns;
    std::set<std::string> m_warnedOperations;
    std::set<std::string> m_fixtureMissing;
    std::vector<std::wstring> m_generateWarnings;
    // The last clearance inputs GenerateOperationMoves worked out, for trimming.
    struct ClearanceInputs
    {
        bool havePartMaxZ = false;
        double partMaxZ = 0.0;
        bool haveStockMaxZ = false;
        double stockMaxZ = 0.0;
        bool haveFixtureMaxZ = false;
        double fixtureMaxZ = 0.0;
    } m_lastClearance;
    // The chosen Fixtures step's outlines, and the parts they were read for.
    std::string m_fixtureEdgesFor;
    std::vector<camcore::LineSegment> m_fixtureEdges;

    // The progress window's helper while Generate runs, or null.
    GenerateProgress* m_pProgress = nullptr;

    // The simulation, while its dialog is open, and its stock's mesh, chunk by chunk and joined.
    CSimulationDlg* m_pSimulationDlg;
    camcore::Simulation m_simulation;
    camcore::StockBox m_simulationBox;                  // the stock it started from, to start again
    SimulationShow m_simulationShow;
    std::vector<camcore::Vec3> m_simulationPartTriangles;   // all the parts, in assembly mm, read once
    bool m_simulationPartsRead = false;
    std::wstring m_simulationPartProblem;
    camcore::VoxelStock m_simulationPart;               // the parts in the simulation's grid
    bool m_simulationPartReady = false;
    bool m_simulationCompareStale = true;
    double m_simulationNotEnough = 0.0;                 // mm^3
    double m_simulationTooMuch = 0.0;
    camcore::StockMesh m_simulationPartMesh;
    // Each Fixtures step's parts as their cells' surface, by the grids the operations share.
    std::map<const void*, camcore::StockMesh> m_simulationFixtureMeshes;
    camcore::StockMesh m_simulationNotEnoughMesh;
    camcore::StockMesh m_simulationTooMuchMesh;
    std::vector<camcore::Vec3> m_simulationTriangles;
    std::vector<camcore::StockMesh> m_simulationChunks;
    camcore::StockMesh m_simulationMesh;

    // The stock's shape, read again when the stock or the model changes.
    bool m_stockShapeValid;
    bool m_stockUnreadable;         // the job's stock couldn't be read when last drawn
    std::vector<camcore::LineSegment> m_stockEdges;
    std::string m_stockFacetsPath;                  // the imported part m_stockPartEdges came from
    std::vector<camcore::LineSegment> m_stockPartEdges;     // in the part's own coordinates, mm
    std::vector<camcore::Vec3> m_stockPartPoints;

    std::filesystem::path m_lastPostPath;   // so the next post suggests the same folder
    std::wstring m_lastPostName;            // and the same post

    // Picking changes Alibre's selection filter. This puts it back.
    bool m_filterChanged;
    SelectionFilter m_previousFilter;
};

/////////////////////////////////////////////////////////////////////////////
// The window Alibre hosts in the Design Explorer tab.

class CCamTabWnd : public CWnd
{
public:
    explicit CCamTabWnd(CCamTabCommand* pTab);

    bool CreateInTab(HWND hParent);
    void DetachFromCommand() { m_pTab = nullptr; }

    void SetStatusText(const std::wstring& text);
    void EnableSave(bool enable);
    void ShowTools(const camcore::ToolLibrary& library, const std::string& selectedToolId);
    void ShowMachine(const std::wstring& name);
    void ShowMaterial(const std::wstring& name);
    void MarkSelectedTool(const camcore::ToolLibrary& library, const std::string& selectedToolId);
    void ShowSequence(const camcore::Job& job, const StepRef& selected);
    void ShowStock(const camcore::Stock& stock);

    // Marks with a red exclamation mark the nodes with a problem: an operation
    // with no toolpath (unless it's suppressed), an incomplete Set Origin, and the
    // machine, material or stock when they can't be found or read. Never folders.
    // warnIds: steps (by ID) with a fixture problem: a Fixtures step that can't be
    // read, or an operation that comes too close to one.
    void ShowWarnings(const camcore::Job& job, bool machine, bool material, bool stock,
                      const std::set<std::string>& warnIds);

    // Starts editing a step's name in place.
    void StartRename(const StepRef& ref);

    // Clears Alibre's selection once the pick that made it has been handled.
    void PostClearSelection();

protected:
    afx_msg int OnCreate(LPCREATESTRUCT pCreate);
    afx_msg void OnDestroy();
    afx_msg void OnSize(UINT type, int cx, int cy);
    afx_msg void OnTimer(UINT_PTR id);
    afx_msg void OnSave();
    afx_msg void OnLoad();
    afx_msg void OnSaveTemplate();
    afx_msg void OnLoadTemplate();
    afx_msg void OnRefresh();
    afx_msg void OnForeground();
    afx_msg void OnShowToolpaths();
    afx_msg void OnShowStock();
    afx_msg void OnShowOrigin();
    afx_msg void OnToolbarTip(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnSettings();
    afx_msg void OnHelp();
    afx_msg void OnTreeSelectionChanged(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnTreeSelectionChanging(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnTreeDoubleClick(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnTreeRightClick(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnTreeBeginRename(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnTreeEndRename(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnTreeBeginDrag(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnTreeCustomDraw(NMHDR* pNotify, LRESULT* pResult);
    afx_msg void OnMouseMove(UINT flags, CPoint point);
    afx_msg void OnLButtonUp(UINT flags, CPoint point);
    afx_msg void OnRButtonDown(UINT flags, CPoint point);
    afx_msg void OnCaptureChanged(CWnd* pWnd);
    afx_msg LRESULT OnLoadJob(WPARAM wParam, LPARAM lParam);
    afx_msg LRESULT OnClearSelection(WPARAM wParam, LPARAM lParam);

    void PostNcDestroy() override;

    DECLARE_MESSAGE_MAP()

private:
    void LayoutControls(int cx, int cy);
    void ApplyViewSize();           // the text and icons at m_settings' size
    HTREEITEM ItemUnderCursor(bool select);
    bool IsStepItem(HTREEITEM hItem) const;
    StepRef StepOfItem(HTREEITEM hItem) const;
    HTREEITEM FindStepItem(const StepRef& ref) const;
    bool IsChosen(const StepRef& ref) const;
    std::vector<StepRef> OperationsBetween(const StepRef& from, const StepRef& to) const;
    void SetChosen(const std::vector<StepRef>& chosen);

    // Dragging. The window holds the mouse capture while a step is dragged.
    struct DropPlace
    {
        bool valid = false;
        StepRef target;
        DropSide side = DropSide::Before;
        HTREEITEM hMarkItem = nullptr;  // where the insert mark or highlight goes
    };
    DropPlace FindDropPlace(CPoint treePoint) const;
    void ShowDropPlace(const DropPlace& place);
    void EndDrag();

    CCamTabCommand* m_pTab;
    CTreeCtrl m_tree;
    CStatic m_status;
    CToolBarCtrl m_toolbar;         // view toggles above the tree
    CToolBarCtrl m_settingsBar;     // the gear, at the right
    CImageList m_toolbarImages;
    CImageList m_treeImages;        // one for each kind of node, at the view size
    CFont m_font;                   // the tab's text, at the view size
    camcore::ProgramSettings m_settings;
    bool m_settingsWritable;        // false if the file is damaged or newer, so it isn't overwritten
    CToolBarCtrl m_fileBar;         // save, load and the templates, at the top
    CToolBarCtrl m_refreshBar;      // refresh, at the top right
    HTREEITEM m_hMachine;
    HTREEITEM m_hTools;
    HTREEITEM m_hMaterial;
    HTREEITEM m_hStock;
    HTREEITEM m_hOperations;
    bool m_fillingTree;
    std::set<HTREEITEM> m_greyItems;    // suppressed steps, and operations in suppressed folders
    std::vector<StepRef> m_chosen;      // operations chosen with Ctrl or Shift, besides the selected one
    StepRef m_anchor;                   // where Shift+click starts from
    bool m_visible;

    bool m_dragging;
    StepRef m_dragFrom;
};
