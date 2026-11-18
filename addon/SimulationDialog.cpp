// SimulationDialog.cpp : the modeless Simulation dialog, which plays the toolpaths
// against the stock in Alibre's view.

#include "stdafx.h"
#include "CAMForAlibre.h"
#include "SimulationDialog.h"
#include "CamTabCommand.h"
#include "DisplaySettings.h"
#include "Help.h"
#include "ModelReader.h"
#include "Log.h"

#include <cmath>

namespace
{
    const UINT_PTR kPlayTimer = 1;
    const UINT kPlayMilliseconds = 50;
    const UINT_PTR kRunTimer = 2;           // To Op End and To End, a tenth of a second of moves a tick
    const UINT kRunMilliseconds = 1;
    const ULONGLONG kRunSliceMilliseconds = 100;
    const int kSpeedSteps = 100;            // the speed slider's range, 0 to this
    const double kMostMovesPerTick = 500.0;

    CPoint g_lastPosition(-1, -1);
    int g_lastSpeed = 40;
    bool g_realTime = false;
    int g_realTimeSpeed = 50;       // the real time slider, 0 to kSpeedSteps: 50 is 1 times
    const ULONGLONG kLongestTick = 250;     // real time: a slow tick counts as no more than this
    bool g_autoDivide = true;
    camcore::StockModel g_stockModel = camcore::StockModel::Voxels;
    bool g_ignoreCollisions = false;    // off at first, remembered until Alibre closes
    bool g_turnWork = false;            // likewise: turn the view with the rotary axis
    std::wstring g_stlFolder;       // where the last STL was saved this session
    CCamTabCommand::SimulationShow g_show;      // what's shown, remembered until Alibre closes
    int g_resolutionStop = -1;      // the slider's stop, fine to coarse; -1 is the middle
    bool g_stopOverload = false;    // Stop when over, remembered until Alibre closes
    double g_overloadPercent = 150.0;

    // A move of the operation whose load to show: the one being cut, or the one just
    // finished when the next hasn't started (as To Op End leaves it), so its graph stays.
    size_t LoadShownMove(const camcore::Simulation& simulation)
    {
        const std::vector<camcore::SimMove>& moves = simulation.Moves();
        const size_t at = (std::min)(simulation.Position(), moves.size() - 1);
        if (at > 0 && at == simulation.Position() && moves[at - 1].operation != moves[at].operation)
            return at - 1;
        return at;
    }

    COLORREF BandColour(camcore::LoadBand band)
    {
        DisplayColour colour = kSimLoadAirColour;
        switch (band)
        {
        case camcore::LoadBand::Under:
            colour = kSimLoadUnderColour;
            break;
        case camcore::LoadBand::Over:
            colour = kSimLoadOverColour;
            break;
        case camcore::LoadBand::Overloaded:
            colour = kSimLoadOverloadedColour;
            break;
        case camcore::LoadBand::NoTarget:
            colour = kSimLoadNoTargetColour;
            break;
        default:
            break;
        }
        return RGB(colour.red, colour.green, colour.blue);
    }

    int StopIndex(const std::vector<double>& stops)
    {
        if (stops.empty())
            return 0;
        if (g_resolutionStop < 0)
            return static_cast<int>(stops.size() / 2);
        return (std::min)(g_resolutionStop, static_cast<int>(stops.size()) - 1);
    }

    // Machining time as m:ss, or h:mm:ss from an hour.
    std::wstring TimeText(double seconds)
    {
        const long long whole = static_cast<long long>(std::llround((std::max)(0.0, seconds)));
        wchar_t text[32];
        if (whole >= 3600)
            swprintf_s(text, L"%lld:%02lld:%02lld", whole / 3600, (whole / 60) % 60, whole % 60);
        else
            swprintf_s(text, L"%lld:%02lld", whole / 60, whole % 60);
        return text;
    }

    std::wstring WithCommas(size_t value)
    {
        std::wstring text = std::to_wstring(value);
        for (int at = static_cast<int>(text.size()) - 3; at > 0; at -= 3)
            text.insert(static_cast<size_t>(at), L",");
        return text;
    }
}

BEGIN_MESSAGE_MAP(CSimulationDlg, CDialog)
    ON_BN_CLICKED(IDC_HELP_BUTTON, &CSimulationDlg::OnHelpButton)
    ON_BN_CLICKED(IDC_SIM_REWIND, &CSimulationDlg::OnRewind)
    ON_BN_CLICKED(IDC_SIM_STEP, &CSimulationDlg::OnStep)
    ON_BN_CLICKED(IDC_SIM_PLAY, &CSimulationDlg::OnPlay)
    ON_BN_CLICKED(IDC_SIM_OPERATION_END, &CSimulationDlg::OnOperationEnd)
    ON_BN_CLICKED(IDC_SIM_END, &CSimulationDlg::OnEnd)
    ON_BN_CLICKED(IDC_SIM_AUTO_DIVIDE, &CSimulationDlg::OnAutoDivide)
    ON_CBN_SELCHANGE(IDC_SIM_MODEL, &CSimulationDlg::OnModelChanged)
    ON_BN_CLICKED(IDC_SIM_IGNORE_COLLISIONS, &CSimulationDlg::OnIgnoreCollisions)
    ON_BN_CLICKED(IDC_SIM_TURN_WORK, &CSimulationDlg::OnTurnWork)
    ON_BN_CLICKED(IDC_SIM_SAVE_STL, &CSimulationDlg::OnSaveStl)
    ON_BN_CLICKED(IDC_SIM_SHOW_STOCK, &CSimulationDlg::OnShowChanged)
    ON_BN_CLICKED(IDC_SIM_SHOW_PART, &CSimulationDlg::OnShowChanged)
    ON_BN_CLICKED(IDC_SIM_SHOW_FIXTURES, &CSimulationDlg::OnShowChanged)
    ON_BN_CLICKED(IDC_SIM_SHOW_NOT_ENOUGH, &CSimulationDlg::OnShowChanged)
    ON_BN_CLICKED(IDC_SIM_SHOW_TOO_MUCH, &CSimulationDlg::OnShowChanged)
    ON_BN_CLICKED(IDC_SIM_REALTIME, &CSimulationDlg::OnRealTimeChanged)
    ON_BN_CLICKED(IDC_SIM_SHOW_LOAD, &CSimulationDlg::OnShowChanged)
    ON_BN_CLICKED(IDC_SIM_STOP_OVERLOAD, &CSimulationDlg::OnStopOverload)
    ON_EN_CHANGE(IDC_SIM_OVERLOAD_PCT, &CSimulationDlg::OnOverloadPercentChanged)
    ON_WM_DRAWITEM()
    ON_WM_HSCROLL()
    ON_WM_TIMER()
    ON_WM_CTLCOLOR()
    ON_WM_CLOSE()
END_MESSAGE_MAP()

void CSimulationDlg::OnHelpButton()
{
    ShowHelp(L"simulation");
}

camcore::SimResolution CSimulationDlg::ChosenResolution(const std::vector<double>& stops)
{
    camcore::SimResolution resolution;
    resolution.model = g_stockModel;
    // Tri-dexels are never auto divided: the slider always sets their spacing.
    resolution.autoDivide = g_autoDivide && g_stockModel == camcore::StockModel::Voxels;
    if (!resolution.autoDivide && !stops.empty())
        resolution.cell = stops[static_cast<size_t>(StopIndex(stops))];
    return resolution;
}

camcore::StockModel CSimulationDlg::ChosenModel()
{
    return g_stockModel;
}

CSimulationDlg::CSimulationDlg(CCamTabCommand* pTab)
    : CDialog(IDD_SIMULATION, nullptr)
    , m_pTab(pTab)
    , m_playing(false)
{
}

bool CSimulationDlg::CreateModeless(CWnd* pParent)
{
    return Create(IDD_SIMULATION, pParent) != FALSE;
}

BOOL CSimulationDlg::OnInitDialog()
{
    CDialog::OnInitDialog();

    m_position.SubclassDlgItem(IDC_SIM_POSITION, this);
    m_speed.SubclassDlgItem(IDC_SIM_SPEED, this);
    m_speed.SetRange(0, kSpeedSteps);
    m_speed.SetPos(g_lastSpeed);
    m_realTimeSpeed.SubclassDlgItem(IDC_SIM_REALTIME_SPEED, this);
    m_realTimeSpeed.SetRange(0, kSpeedSteps);
    m_realTimeSpeed.SetPos(g_realTimeSpeed);
    CheckDlgButton(IDC_SIM_REALTIME, g_realTime ? BST_CHECKED : BST_UNCHECKED);

    // The stock model, then the resolution: Auto divide, or a stop from fine to coarse.
    CComboBox* pModel = static_cast<CComboBox*>(GetDlgItem(IDC_SIM_MODEL));
    pModel->AddString(L"Voxels");
    pModel->AddString(L"Tri-dexel");
    pModel->SetCurSel(g_stockModel == camcore::StockModel::TriDexel ? 1 : 0);
    m_resolution.SubclassDlgItem(IDC_SIM_RESOLUTION, this);
    SetStops();
    CheckDlgButton(IDC_SIM_AUTO_DIVIDE, g_autoDivide ? BST_CHECKED : BST_UNCHECKED);
    ShowResolution();
    CheckDlgButton(IDC_SIM_IGNORE_COLLISIONS, g_ignoreCollisions ? BST_CHECKED : BST_UNCHECKED);
    if (m_pTab != nullptr)
        m_pTab->GetSimulation().SetIgnoreCollisions(g_ignoreCollisions);

    // Turn the work: only on a 4 axis machine with a work zero.
    const bool canTurn = m_pTab != nullptr && m_pTab->CanTurnSimulationView();
    GetDlgItem(IDC_SIM_TURN_WORK)->EnableWindow(canTurn ? TRUE : FALSE);
    CheckDlgButton(IDC_SIM_TURN_WORK, (canTurn && g_turnWork) ? BST_CHECKED : BST_UNCHECKED);
    if (m_pTab != nullptr)
        m_pTab->SetSimulationTurnsWork(canTurn && g_turnWork);

    // What's shown.
    CheckDlgButton(IDC_SIM_SHOW_STOCK, g_show.stock ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(IDC_SIM_SHOW_PART, g_show.part ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(IDC_SIM_SHOW_FIXTURES, g_show.fixtures ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(IDC_SIM_SHOW_NOT_ENOUGH, g_show.notEnough ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(IDC_SIM_SHOW_TOO_MUCH, g_show.tooMuch ? BST_CHECKED : BST_UNCHECKED);
    CheckDlgButton(IDC_SIM_SHOW_LOAD, g_show.load ? BST_CHECKED : BST_UNCHECKED);
    if (m_pTab != nullptr)
        m_pTab->SetSimulationShow(g_show);

    // Stopping when the tool is overloaded.
    CheckDlgButton(IDC_SIM_STOP_OVERLOAD, g_stopOverload ? BST_CHECKED : BST_UNCHECKED);
    wchar_t percent[32];
    swprintf_s(percent, L"%g", g_overloadPercent);
    SetDlgItemText(IDC_SIM_OVERLOAD_PCT, percent);
    ApplyOverload();
    m_ready = true;

    if (m_pTab != nullptr)
    {
        const camcore::Simulation& simulation = m_pTab->GetSimulation();
        m_position.SetRange(0, static_cast<int>((std::min)(simulation.Moves().size(), size_t(0x7FFFFFFF))));
    }

    if (g_lastPosition.x >= 0)
        SetWindowPos(nullptr, g_lastPosition.x, g_lastPosition.y, 0, 0, SWP_NOSIZE | SWP_NOZORDER);
    ShowState();
    return TRUE;
}

void CSimulationDlg::PostNcDestroy()
{
    delete this;
}

size_t CSimulationDlg::MovesPerTick() const
{
    // From one move a tick at the left to the most at the right, on a log scale.
    const double fraction = static_cast<double>(m_speed.GetPos()) / kSpeedSteps;
    return static_cast<size_t>(std::lround(std::pow(kMostMovesPerTick, fraction)));
}

double CSimulationDlg::RealTimeFactor() const
{
    // 0.1 times at the left, 1 in the middle, 10 at the right, on a log scale.
    const double fraction = static_cast<double>(m_realTimeSpeed.GetPos()) / kSpeedSteps;
    return 0.1 * std::pow(100.0, fraction);
}

void CSimulationDlg::OnRealTimeChanged()
{
    g_realTime = IsDlgButtonChecked(IDC_SIM_REALTIME) == BST_CHECKED;
    m_lastTick = GetTickCount64();
    ShowState();
}

void CSimulationDlg::Stop()
{
    if (m_playing)
    {
        KillTimer(kPlayTimer);
        if (m_pTab != nullptr)
            m_pTab->LogSimulationTiming(L"play", GetTickCount64() - m_runStarted);
    }
    m_playing = false;
    if (m_running)
    {
        // Stopped part way: the view catches up with where it got to.
        KillTimer(kRunTimer);
        m_running = false;
        if (m_pTab != nullptr)
        {
            m_pTab->OnSimulationChanged();
            m_pTab->LogSimulationTiming(L"run", GetTickCount64() - m_runStarted);
        }
    }
    SetDlgItemText(IDC_SIM_PLAY, L"Play");
}

void CSimulationDlg::Changed()
{
    if (m_pTab != nullptr)
        m_pTab->OnSimulationChanged();
    ShowState();
}

void CSimulationDlg::ShowState()
{
    if (m_pTab == nullptr)
        return;
    const camcore::Simulation& simulation = m_pTab->GetSimulation();

    // Where it is: the operation, and the move of all of them.
    std::wstring status;
    const camcore::SimOperation* pOperation = simulation.CurrentOperation();
    if (pOperation != nullptr)
        status = FromUtf8(pOperation->name) + L" \u2014 ";
    status += L"move " + WithCommas(simulation.Position()) + L" of " + WithCommas(simulation.Moves().size());
    status += L" \u2014 time " + TimeText(simulation.ElapsedSeconds()) + L" of " + TimeText(simulation.TotalSeconds());
    if (simulation.Finished())
        status += L" (done)";
    SetDlgItemText(IDC_SIM_STATUS, status.c_str());
    SetDlgItemText(IDC_SIM_TOOL, (pOperation != nullptr) ? (L"Tool: " + FromUtf8(pOperation->tool)).c_str() : L"");
    m_position.SetPos(static_cast<int>((std::min)(simulation.Position(), size_t(0x7FFFFFFF))));

    // The tool's load: the move it's on (or the last one cut), and the operation so far.
    std::wstring loadText, summary;
    const camcore::MoveLoad* pLoad = simulation.CurrentLoad();
    if (pLoad != nullptr && !simulation.Moves().empty())
    {
        const size_t index = static_cast<size_t>(pLoad - simulation.MoveLoads().data());
        const camcore::SimOperation& loadOperation = simulation.Operations()[simulation.Moves()[index].operation];
        loadText = L"Load: " + FromUtf8(camcore::MoveLoadText(*pLoad, loadOperation.load));
        if (loadOperation.load.chipLoadTarget <= 0.0 && pLoad->engaged)
            loadText += L" (the tool has no chip load to compare with)";
        if (pLoad->powerPercent > 100.0)
            loadText += L" \u2014 more than the spindle's power";
        const size_t operation = simulation.Moves()[LoadShownMove(simulation)].operation;
        summary = L"This operation: " + FromUtf8(camcore::OperationLoadText(simulation.OperationLoads()[operation],
                                                                               simulation.Operations()[operation].load));
    }
    SetDlgItemText(IDC_SIM_LOAD, loadText.c_str());
    SetDlgItemText(IDC_SIM_LOAD_SUMMARY, summary.c_str());
    GetDlgItem(IDC_SIM_LOAD_GRAPH)->Invalidate(FALSE);

    SetDlgItemText(IDC_SIM_INFO, (m_saved.empty() ? m_note : m_saved + L" " + m_note).c_str());
    SetDlgItemText(IDC_SIM_COMPARE, m_pTab->SimulationCompareText().c_str());

    const size_t speed = MovesPerTick();
    SetDlgItemText(IDC_SIM_SPEED_TEXT, (WithCommas(speed) + (speed == 1 ? L" move a step" : L" moves a step")).c_str());

    // Real time plays at the feeds, times the slider; moves a step aren't used.
    const double factor = RealTimeFactor();
    wchar_t times[32];
    swprintf_s(times, L"%.2g\u00D7 real time", factor);
    SetDlgItemText(IDC_SIM_REALTIME_TEXT, times);
    GetDlgItem(IDC_SIM_REALTIME_SPEED)->EnableWindow(g_realTime ? TRUE : FALSE);
    GetDlgItem(IDC_SIM_REALTIME_TEXT)->EnableWindow(g_realTime ? TRUE : FALSE);
    GetDlgItem(IDC_SIM_SPEED)->EnableWindow(g_realTime ? FALSE : TRUE);
    GetDlgItem(IDC_SIM_SPEED_TEXT)->EnableWindow(g_realTime ? FALSE : TRUE);

    std::wstring warning;
    if (simulation.HasWarning())
    {
        const camcore::SimMove& move = simulation.Moves()[simulation.WarningMove()];
        const wchar_t* what = L"Rapid into stock";
        if (simulation.Warning() == camcore::SimWarning::HolderHitsStock)
            what = L"The tool holder hits the stock";
        else if (simulation.Warning() == camcore::SimWarning::ShankHitsStock)
            what = L"The tool's shank hits the stock";
        std::wstring fixture;
        if (simulation.Warning() == camcore::SimWarning::ToolHitsFixture)
            fixture = L"The tool hits fixture " + FromUtf8(simulation.WarningFixture());
        else if (simulation.Warning() == camcore::SimWarning::HolderHitsFixture)
            fixture = L"The tool holder hits fixture " + FromUtf8(simulation.WarningFixture());
        else if (simulation.Warning() == camcore::SimWarning::ToolOverloaded)
        {
            wchar_t text[96];
            swprintf_s(text, L"The tool is overloaded (%.0f%% of its chip load)",
                       simulation.MoveLoads()[simulation.WarningMove()].percent);
            fixture = text;
        }
        if (!fixture.empty())
            what = fixture.c_str();
        warning = std::wstring(what) + L" in " + FromUtf8(simulation.Operations()[move.operation].name) +
                  L", toolpath move " + std::to_wstring(move.toolpathMove + 1) + L".";
    }
    SetDlgItemText(IDC_SIM_WARNING, warning.c_str());

    const BOOL more = simulation.Finished() ? FALSE : TRUE;
    GetDlgItem(IDC_SIM_STEP)->EnableWindow(more);
    GetDlgItem(IDC_SIM_OPERATION_END)->EnableWindow(more);
    GetDlgItem(IDC_SIM_END)->EnableWindow(more);
    GetDlgItem(IDC_SIM_PLAY)->EnableWindow(more || m_playing || m_running);
}

void CSimulationDlg::OnShowChanged()
{
    g_show.stock = IsDlgButtonChecked(IDC_SIM_SHOW_STOCK) == BST_CHECKED;
    g_show.part = IsDlgButtonChecked(IDC_SIM_SHOW_PART) == BST_CHECKED;
    g_show.fixtures = IsDlgButtonChecked(IDC_SIM_SHOW_FIXTURES) == BST_CHECKED;
    g_show.notEnough = IsDlgButtonChecked(IDC_SIM_SHOW_NOT_ENOUGH) == BST_CHECKED;
    g_show.tooMuch = IsDlgButtonChecked(IDC_SIM_SHOW_TOO_MUCH) == BST_CHECKED;
    g_show.load = IsDlgButtonChecked(IDC_SIM_SHOW_LOAD) == BST_CHECKED;
    if (m_pTab == nullptr)
        return;
    CWaitCursor wait;
    m_pTab->SetSimulationShow(g_show);
    ShowState();
}

void CSimulationDlg::OnStopOverload()
{
    g_stopOverload = IsDlgButtonChecked(IDC_SIM_STOP_OVERLOAD) == BST_CHECKED;
    ApplyOverload();
    Changed();
}

void CSimulationDlg::OnOverloadPercentChanged()
{
    if (!m_ready)
        return;
    // Taken as it's typed, when it's a number over 0; otherwise the last one stays.
    CString text;
    GetDlgItemText(IDC_SIM_OVERLOAD_PCT, text);
    text.Trim();
    wchar_t* pEnd = nullptr;
    const double value = wcstod(text, &pEnd);
    if (text.IsEmpty() || pEnd == nullptr || *pEnd != L'\0' || !(value > 0.0))
        return;
    g_overloadPercent = value;
    ApplyOverload();
    Changed();
}

void CSimulationDlg::ApplyOverload()
{
    GetDlgItem(IDC_SIM_OVERLOAD_PCT)->EnableWindow(g_stopOverload ? TRUE : FALSE);
    GetDlgItem(IDC_SIM_OVERLOAD_LABEL)->EnableWindow(g_stopOverload ? TRUE : FALSE);
    if (m_pTab != nullptr)
        m_pTab->GetSimulation().SetStopOnOverload(g_stopOverload, g_overloadPercent);
}

void CSimulationDlg::OnDrawItem(int id, LPDRAWITEMSTRUCT pDraw)
{
    if (id != IDC_SIM_LOAD_GRAPH)
    {
        CDialog::OnDrawItem(id, pDraw);
        return;
    }
    // Drawn off screen, then copied, so it doesn't flicker as it plays.
    CDC* pDC = CDC::FromHandle(pDraw->hDC);
    const CRect rect = pDraw->rcItem;
    CDC memory;
    memory.CreateCompatibleDC(pDC);
    CBitmap bitmap;
    bitmap.CreateCompatibleBitmap(pDC, rect.Width(), rect.Height());
    CBitmap* pOldBitmap = memory.SelectObject(&bitmap);
    CFont* pOldFont = memory.SelectObject(GetFont());
    DrawLoadGraph(memory, CRect(0, 0, rect.Width(), rect.Height()));
    pDC->BitBlt(rect.left, rect.top, rect.Width(), rect.Height(), &memory, 0, 0, SRCCOPY);
    memory.SelectObject(pOldFont);
    memory.SelectObject(pOldBitmap);
}

void CSimulationDlg::DrawLoadGraph(CDC& dc, const CRect& rect)
{
    dc.FillSolidRect(rect, RGB(255, 255, 255));
    dc.SetBkMode(TRANSPARENT);
    const int width = rect.Width() - 2, height = rect.Height() - 2;
    if (m_pTab == nullptr || width <= 0 || height <= 0)
        return;
    const camcore::Simulation& simulation = m_pTab->GetSimulation();
    const std::vector<camcore::SimMove>& moves = simulation.Moves();
    const std::vector<camcore::MoveLoad>& loads = simulation.MoveLoads();
    if (moves.empty() || loads.size() != moves.size())
        return;

    // The current operation's moves, across the graph by machining time.
    const size_t at = LoadShownMove(simulation);
    const size_t operation = moves[at].operation;
    size_t begin = at, end = at;
    while (begin > 0 && moves[begin - 1].operation == operation)
        --begin;
    while (end < moves.size() && moves[end].operation == operation)
        ++end;
    double total = 0.0;
    for (size_t m = begin; m < end; ++m)
        total += moves[m].seconds;
    const bool byTime = total > 0.0;
    if (!byTime)
        total = static_cast<double>(end - begin);

    // Each column of pixels: the highest chip, as a percent of the chip load (or in
    // mm when the tool has none), as a bar, and the highest power, as a percent of the
    // spindle's, as a line over the bars.
    const camcore::ToolLoadSetup& setup = simulation.Operations()[operation].load;
    const bool power = setup.spindlePower > 0.0 && setup.cuttingForce > 0.0;
    const bool percent = setup.chipLoadTarget > 0.0;
    const double limit = simulation.OverloadPercent();
    std::vector<double> top(static_cast<size_t>(width), -1.0);
    std::vector<double> powerTop(static_cast<size_t>(width), -1.0);
    std::vector<camcore::LoadBand> bands(static_cast<size_t>(width), camcore::LoadBand::NotCutting);
    double highest = 0.0, highestPower = 0.0, t = 0.0, now = 0.0;
    for (size_t m = begin; m < end; ++m)
    {
        const double seconds = byTime ? moves[m].seconds : 1.0;
        if (m < simulation.Position())
            now = t + seconds;
        const camcore::MoveLoad& load = loads[m];
        if (load.measured)
        {
            const double value = !load.engaged ? 0.0 : (percent ? load.percent : load.chip);
            const double powerValue = (std::max)(0.0, load.powerPercent);
            // The bar's colour by the chip alone; the power has its own line.
            camcore::MoveLoad chipOnly = load;
            chipOnly.powerPercent = -1.0;
            const camcore::LoadBand band = camcore::BandOf(chipOnly, limit);
            const int x0 = (std::min)(width - 1, static_cast<int>(t / total * width));
            const int x1 = (std::min)(width - 1, (std::max)(x0, static_cast<int>((t + seconds) / total * width) - 1));
            for (int x = x0; x <= x1; ++x)
            {
                const size_t column = static_cast<size_t>(x);
                if (value > top[column])
                {
                    top[column] = value;
                    bands[column] = band;
                }
                powerTop[column] = (std::max)(powerTop[column], powerValue);
            }
            highest = (std::max)(highest, value);
            highestPower = (std::max)(highestPower, powerValue);
        }
        t += seconds;
    }

    // The bars' scale fits the chips; the power shares it when the chips are percents,
    // and is cut off at the top when it's higher (the label gives its highest).
    const double scale = percent ? (std::max)({ 120.0, limit * 1.15, highest * 1.05 })
                                 : (std::max)(highest * 1.1, 1e-6);
    const double powerScale = percent ? scale : (std::max)(120.0, (std::min)(highestPower * 1.05, 300.0));
    const auto yOf = [&](double value, double of) {
        const LONG y = rect.bottom - 1 - static_cast<LONG>(value / of * height);
        return (std::max)(rect.top + 1, y);
    };
    for (int x = 0; x < width; ++x)
    {
        const double value = top[static_cast<size_t>(x)];
        if (value < 0.0)
            continue;
        const int y = (std::min)(yOf(value, scale), rect.bottom - 2);       // air shows as a line at the bottom
        dc.FillSolidRect(rect.left + 1 + x, y, 1, rect.bottom - 1 - y, BandColour(bands[static_cast<size_t>(x)]));
    }

    // 100% and the stop limit, dotted.
    if (percent)
    {
        CPen hundred(PS_DOT, 1, RGB(0, 120, 40));
        CPen stop(PS_DOT, 1, RGB(200, 0, 0));
        CPen* pOld = dc.SelectObject(&hundred);
        dc.MoveTo(rect.left + 1, yOf(100.0, scale));
        dc.LineTo(rect.right - 1, yOf(100.0, scale));
        if (simulation.StopsOnOverload())
        {
            dc.SelectObject(&stop);
            dc.MoveTo(rect.left + 1, yOf(limit, scale));
            dc.LineTo(rect.right - 1, yOf(limit, scale));
        }
        dc.SelectObject(pOld);
    }

    // The power line, purple, and the spindle's 100% when the bars are in mm.
    const COLORREF kPowerColour = RGB(130, 0, 170);
    if (power)
    {
        CPen dotted(PS_DOT, 1, kPowerColour);
        CPen line(PS_SOLID, 2, kPowerColour);
        CPen* pOld = dc.SelectObject(&dotted);
        if (!percent)
        {
            dc.MoveTo(rect.left + 1, yOf(100.0, powerScale));
            dc.LineTo(rect.right - 1, yOf(100.0, powerScale));
        }
        dc.SelectObject(&line);
        bool drawing = false;
        for (int x = 0; x < width; ++x)
        {
            const double value = powerTop[static_cast<size_t>(x)];
            if (value < 0.0)
            {
                drawing = false;
                continue;
            }
            const int y = (std::min)(yOf(value, powerScale), rect.bottom - 2);
            if (drawing)
                dc.LineTo(rect.left + 1 + x, y);
            else
                dc.MoveTo(rect.left + 1 + x, y);
            drawing = true;
        }
        dc.SelectObject(pOld);
    }

    // Where the simulation has got to.
    CPen cursor(PS_SOLID, 1, RGB(60, 60, 60));
    CPen* pOld = dc.SelectObject(&cursor);
    const int x = rect.left + 1 + (std::min)(width - 1, static_cast<int>(now / total * width));
    dc.MoveTo(x, rect.top);
    dc.LineTo(x, rect.bottom);
    dc.SelectObject(pOld);

    wchar_t label[160];
    if (percent && setup.targetProgrammed)
        swprintf_s(label, L"Chip, %% of the programmed chip (up to %.0f%%)", highest);
    else if (percent)
        swprintf_s(label, L"Chip, %% of the chip load (up to %.0f%%)", highest);
    else
        swprintf_s(label, L"Chip, mm (up to %.4g)", highest);
    std::wstring text = label;
    if (power)
    {
        swprintf_s(label, L"; power, %% of the spindle (up to %.0f%%)", highestPower);
        text += label;
    }
    dc.SetTextColor(RGB(80, 80, 80));
    dc.TextOut(rect.left + 3, rect.top + 1, text.c_str());
    CBrush frame(RGB(160, 160, 160));
    dc.FrameRect(rect, &frame);
}

void CSimulationDlg::OnSaveStl()
{
    Stop();
    if (m_pTab == nullptr)
        return;

    // The cut stock as it is now, as an STL file: beside the .cam file at first,
    // then in the last folder used.
    const std::filesystem::path suggested = m_pTab->SimulationStlPath();
    const std::wstring folder = g_stlFolder.empty() ? suggested.parent_path().wstring() : g_stlFolder;
    CFileDialog dialog(FALSE, L"stl", suggested.filename().c_str(),
                       OFN_OVERWRITEPROMPT | OFN_HIDEREADONLY | OFN_PATHMUSTEXIST,
                       L"STL files (*.stl)|*.stl|All files (*.*)|*.*||", this);
    if (!folder.empty())
        dialog.m_ofn.lpstrInitialDir = folder.c_str();
    if (dialog.DoModal() != IDOK)
        return;

    const std::filesystem::path path(dialog.GetPathName().GetString());
    g_stlFolder = path.parent_path().wstring();
    m_pTab->SaveSimulationStl(path, m_saved);
    ShowState();
}

void CSimulationDlg::OnAutoDivide()
{
    g_autoDivide = IsDlgButtonChecked(IDC_SIM_AUTO_DIVIDE) == BST_CHECKED;
    ShowResolution();
    Restart();
}

void CSimulationDlg::OnIgnoreCollisions()
{
    g_ignoreCollisions = IsDlgButtonChecked(IDC_SIM_IGNORE_COLLISIONS) == BST_CHECKED;
    if (m_pTab == nullptr)
        return;
    // From the next move on; a warning showing now is cleared when they're ignored.
    camcore::Simulation& simulation = m_pTab->GetSimulation();
    simulation.SetIgnoreCollisions(g_ignoreCollisions);
    if (g_ignoreCollisions && simulation.HasWarning())
    {
        simulation.ClearWarning();
        Changed();
        return;
    }
    ShowState();
}

void CSimulationDlg::OnTurnWork()
{
    g_turnWork = IsDlgButtonChecked(IDC_SIM_TURN_WORK) == BST_CHECKED;
    if (m_pTab != nullptr)
        m_pTab->SetSimulationTurnsWork(g_turnWork);
}

void CSimulationDlg::OnModelChanged()
{
    const int chosen = static_cast<CComboBox*>(GetDlgItem(IDC_SIM_MODEL))->GetCurSel();
    const camcore::StockModel model = (chosen == 1) ? camcore::StockModel::TriDexel : camcore::StockModel::Voxels;
    if (model == g_stockModel)
        return;
    g_stockModel = model;
    SetStops();
    ShowResolution();
    Restart();
}

void CSimulationDlg::SetStops()
{
    m_stops.clear();
    if (m_pTab != nullptr)
        m_stops = m_pTab->SimulationStops(g_stockModel);
    m_resolution.SetRange(0, (std::max)(1, static_cast<int>(m_stops.size()) - 1), TRUE);
    m_resolution.ClearTics(TRUE);
    m_resolution.SetTicFreq(1);
    m_appliedStop = StopIndex(m_stops);
    m_resolution.SetPos(m_appliedStop);
}

void CSimulationDlg::ShowResolution()
{
    const bool dexels = g_stockModel == camcore::StockModel::TriDexel;
    GetDlgItem(IDC_SIM_AUTO_DIVIDE)->EnableWindow(dexels ? FALSE : TRUE);
    const BOOL slider = ((dexels || !g_autoDivide) && m_stops.size() > 1) ? TRUE : FALSE;
    for (int id : { IDC_SIM_RESOLUTION, IDC_SIM_RESOLUTION_LABEL, IDC_SIM_FINE, IDC_SIM_MEDIUM, IDC_SIM_COARSE })
        GetDlgItem(id)->EnableWindow(slider);
}

void CSimulationDlg::Restart()
{
    Stop();
    if (m_pTab == nullptr)
        return;
    CWaitCursor wait;
    m_pTab->RestartSimulation(ChosenResolution(m_stops));
    ShowState();
}

void CSimulationDlg::OnRewind()
{
    Stop();
    if (m_pTab == nullptr)
        return;
    m_pTab->GetSimulation().Rewind();
    Changed();
}

void CSimulationDlg::OnStep()
{
    Stop();
    if (m_pTab == nullptr)
        return;
    m_pTab->GetSimulation().ClearWarning();
    m_pTab->GetSimulation().Step(1);
    Changed();
}

void CSimulationDlg::OnOperationEnd()
{
    Stop();
    if (m_pTab == nullptr)
        return;
    m_pTab->GetSimulation().ClearWarning();
    RunTo(m_pTab->GetSimulation().OperationEnd());
}

void CSimulationDlg::OnEnd()
{
    Stop();
    if (m_pTab == nullptr)
        return;
    m_pTab->GetSimulation().ClearWarning();
    RunTo(m_pTab->GetSimulation().Moves().size());
}

void CSimulationDlg::RunTo(size_t end)
{
    // On a timer, so the dialog's buttons still work: Play becomes Pause. The view
    // is redrawn when it gets there or is stopped; the slider shows where it is.
    if (m_pTab == nullptr || m_pTab->GetSimulation().Position() >= end)
        return;
    m_runEnd = end;
    m_running = true;
    m_runStarted = GetTickCount64();
    m_pTab->ResetSimulationTiming();
    SetDlgItemText(IDC_SIM_PLAY, L"Pause");
    ShowState();
    SetTimer(kRunTimer, kRunMilliseconds, nullptr);
}

void CSimulationDlg::RunTick()
{
    if (m_pTab == nullptr)
    {
        Stop();
        return;
    }
    camcore::Simulation& simulation = m_pTab->GetSimulation();
    const ULONGLONG start = GetTickCount64();
    while (simulation.Position() < m_runEnd && !simulation.HasWarning() &&
           GetTickCount64() - start < kRunSliceMilliseconds)
    {
        if (simulation.Step(1) == 0)
            break;
    }
    if (simulation.Position() >= m_runEnd || simulation.HasWarning() || simulation.Finished())
    {
        Stop();             // redraws the view
        ShowState();
        return;
    }
    ShowState();
}

void CSimulationDlg::OnPlay()
{
    if (m_playing || m_running)
    {
        Stop();
        ShowState();
        return;
    }
    if (m_pTab == nullptr || m_pTab->GetSimulation().Finished())
        return;
    m_pTab->GetSimulation().ClearWarning();
    m_playing = true;
    m_lastTick = GetTickCount64();
    m_runStarted = m_lastTick;
    m_pTab->ResetSimulationTiming();
    SetDlgItemText(IDC_SIM_PLAY, L"Pause");
    SetTimer(kPlayTimer, kPlayMilliseconds, nullptr);
}

void CSimulationDlg::OnTimer(UINT_PTR id)
{
    if (id == kRunTimer)
    {
        RunTick();
        return;
    }
    if (id != kPlayTimer)
    {
        CDialog::OnTimer(id);
        return;
    }
    if (m_pTab == nullptr)
    {
        Stop();
        return;
    }
    camcore::Simulation& simulation = m_pTab->GetSimulation();
    if (g_realTime)
    {
        // As much machining time as has passed since the last tick, times the slider.
        const ULONGLONG now = GetTickCount64();
        const ULONGLONG passed = (std::min)(kLongestTick, now - m_lastTick);
        m_lastTick = now;
        simulation.Advance(static_cast<double>(passed) / 1000.0 * RealTimeFactor());
    }
    else
    {
        simulation.Step(MovesPerTick());
    }
    if (simulation.Finished() || simulation.HasWarning())
        Stop();
    Changed();
}

void CSimulationDlg::OnHScroll(UINT code, UINT position, CScrollBar* pScrollBar)
{
    CDialog::OnHScroll(code, position, pScrollBar);
    if (pScrollBar == nullptr || m_pTab == nullptr)
        return;

    if (pScrollBar->GetSafeHwnd() == m_speed.GetSafeHwnd())
    {
        g_lastSpeed = m_speed.GetPos();
        ShowState();
        return;
    }
    if (pScrollBar->GetSafeHwnd() == m_realTimeSpeed.GetSafeHwnd())
    {
        g_realTimeSpeed = m_realTimeSpeed.GetPos();
        ShowState();
        return;
    }

    // A new resolution starts again when the slider is let go at a different stop.
    if (pScrollBar->GetSafeHwnd() == m_resolution.GetSafeHwnd())
    {
        if (code != TB_THUMBTRACK && m_resolution.GetPos() != m_appliedStop)
        {
            g_resolutionStop = m_resolution.GetPos();
            m_appliedStop = g_resolutionStop;
            Restart();
        }
        return;
    }

    // The scrub bar seeks when it's let go, not while it's dragged: seeking back
    // replays from the start.
    if (pScrollBar->GetSafeHwnd() == m_position.GetSafeHwnd() && code != TB_THUMBTRACK)
    {
        Stop();
        CWaitCursor wait;
        m_pTab->GetSimulation().SeekTo(static_cast<size_t>((std::max)(0, m_position.GetPos())));
        Changed();
    }
}

HBRUSH CSimulationDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT ctlColor)
{
    HBRUSH brush = CDialog::OnCtlColor(pDC, pWnd, ctlColor);
    if (pWnd != nullptr && pWnd->GetDlgCtrlID() == IDC_SIM_WARNING)
        pDC->SetTextColor(RGB(200, 0, 0));
    return brush;
}

void CSimulationDlg::OnOK()
{
    Finish();
}

void CSimulationDlg::OnCancel()
{
    Finish();
}

void CSimulationDlg::OnClose()
{
    Finish();
}

void CSimulationDlg::Finish()
{
    Stop();
    CRect rect;
    GetWindowRect(&rect);
    g_lastPosition = rect.TopLeft();
    if (m_pTab != nullptr)
    {
        CCamTabCommand* pTab = m_pTab;
        m_pTab = nullptr;
        pTab->OnSimulationDialogDone(this);
    }
    DestroyWindow();
}
