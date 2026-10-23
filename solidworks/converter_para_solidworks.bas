Attribute VB_Name = "converter_para_solidworks"
' ---------------------------------------------------------------------------
' converter_para_solidworks.bas
'
' PURPOSE
'   Turns the STEP files of this folder into native SolidWorks files:
'     pass 1  pecas\*.step              ->  sldprt\*.SLDPRT   (one file per part)
'     pass 2  Cockpit_2DOF_F1.step      ->  sldasm\Cockpit_2DOF_F1.SLDASM
'
' HOW TO USE
'   1. Open SolidWorks (any document or none).
'   2. Tools > Macro > Run... and choose this file. Keep it in the "solidworks"
'      folder: the macro reads the STEP files next to itself.
'   3. Tools > Options > Import > STEP must have "Import as assembly" / solid bodies
'      active (default). The macro does not change the user options.
'   4. When SolidWorks asks to save the referenced parts of the assembly, accept.
'
' NOTES
'   - Written without access to a SolidWorks installation. It uses only
'     documented calls (LoadFile4, Extension.SaveAs3, CloseDoc). If a call fails
'     the macro writes the file name and the error code to the Immediate window
'     and goes on with the next file.
'   - Pass 2 saves its own copy of every component beside the .SLDASM
'     (names come from the STEP product names). Use the files of pass 1 when you
'     need one stand-alone .SLDPRT per part, and the .SLDASM of pass 2 when you
'     need the integrated assembly. Do not mix them without "Replace Components".
'   - Mates are not part of STEP. Use juntas.csv to add the hinges and slides.
' ---------------------------------------------------------------------------
Option Explicit

Dim swApp As SldWorks.SldWorks

Sub main()
    Dim root As String, f As String, n As Long, bad As Long

    Set swApp = Application.SldWorks
    root = swApp.GetCurrentMacroPathFolder() & "\"
    MakeFolder root & "sldprt"
    MakeFolder root & "sldasm"

    ' ---- pass 1: one .SLDPRT for each STEP part ----
    f = Dir(root & "pecas\*.step")
    Do While f <> ""
        If ConvertFile(root & "pecas\" & f, root & "sldprt\" & Left(f, Len(f) - 5) & ".SLDPRT", 0) Then
            n = n + 1
        Else
            bad = bad + 1
        End If
        f = Dir()
    Loop

    ' ---- pass 2: the assembly ----
    If ConvertFile(root & "Cockpit_2DOF_F1.step", root & "sldasm\Cockpit_2DOF_F1.SLDASM", swSaveAsOptions_SaveReferenced) Then
        n = n + 1
    Else
        bad = bad + 1
    End If

    MsgBox "Done: " & n & " file(s) converted, " & bad & " failed. See the Immediate window for details."
End Sub

Private Sub MakeFolder(ByVal path As String)
    If Dir(path, vbDirectory) = "" Then MkDir path
End Sub

' Opens a STEP file and saves it as a native file. Returns True on success.
Private Function ConvertFile(ByVal src As String, ByVal dst As String, ByVal extraOptions As Long) As Boolean
    Dim model As SldWorks.ModelDoc2
    Dim errs As Long, warns As Long, ok As Boolean

    ConvertFile = False
    Set model = swApp.LoadFile4(src, "r", Nothing, errs)
    If model Is Nothing Then
        Debug.Print "FAILED to open " & src & " (error " & errs & ")"
        Exit Function
    End If
    ok = model.Extension.SaveAs3(dst, swSaveAsVersion_e.swSaveAsCurrentVersion, _
            swSaveAsOptions_e.swSaveAsOptions_Silent + extraOptions, Nothing, Nothing, errs, warns)
    If Not ok Then Debug.Print "FAILED to save " & dst & " (errors " & errs & ", warnings " & warns & ")"
    swApp.CloseDoc model.GetTitle
    ConvertFile = ok
End Function
