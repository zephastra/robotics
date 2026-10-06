"""Gate physical handover on the existing resource/transfer contracts.

This coordinator neither fabricates sensor evidence nor moves actuators itself.
Before motion, both ends are reserved and custody is TRANSFERRING. After motion,
missing confirmation keeps both locks; a timer or an executor's success is not
receiver confirmation. Resource release is a separate measured-clear operation.
"""
from workcell.schema import SchemaRefused
from workcell.transfer import TransferLedger


class PhysicalTransferSession:
    def __init__(self, *, ledger, resources, journal):
        if not isinstance(ledger,TransferLedger):
            raise TypeError('existing TransferLedger required')
        self.ledger,self.resources,self.journal=ledger,resources,journal

    def _save(self, record, event):
        self.journal(dict(event=event,transaction=record.to_dict(),
            resources={name:self.resources.snapshot(name) for name in
                       (record.source,record.receiver)}))

    def execute(self, request, *, now_s, launch_evidence, drive, confirm, source_grant=None):
        if not launch_evidence:
            raise SchemaRefused('REFUSED_MISSING_FIELD','launch evidence required')
        record=self.ledger.request(request,resources=self.resources,now_s=now_s,
                                   source_grant=source_grant)
        self._save(record,'reserved_before_motion')
        # These are the physically engaged docking ends, not stock inventory.
        # RESERVED alone can expire while a tray is half-way across the seam.
        for name in (record.source,record.receiver):
            grant=self.resources.snapshot(name)
            self.resources.confirm_occupied(name,owner=record.transfer_id,
                generation=grant['generation'],epoch=self.resources.epoch,now_s=now_s)
        for _ in range(3):
            self.ledger.advance(record.transfer_id,evidence=launch_evidence)
        self._save(record,'transferring_before_motion')
        try:
            result=drive()
            # Confirmation must independently measure BOTH sides after motion.
            observed=confirm(result)
            required=('receiver_supported','source_cleared','stopped_confirmed')
            if result.get('status')!='SUCCEEDED' or any(
                    observed.get(k) is not True for k in required) or not observed.get('evidence'):
                raise RuntimeError('TRANSFER_CONFIRMATION_UNKNOWN_OR_FAILED')
            for _ in range(2):
                self.ledger.advance(record.transfer_id,evidence=observed['evidence'])
            self._save(record,'committed_on_measured_confirmation')
            return result
        except Exception:
            if record.stage=='TRANSFERRING':
                self.ledger.interrupt(record.transfer_id,reason_code='TRANSFER_UNKNOWN',
                                      evidence=launch_evidence)
                self.ledger.resolve(record.transfer_id,non_transfer_proven=False,
                                    evidence=launch_evidence)
                self._save(record,'attention_locks_retained')
            raise

    def release(self, transfer_id, *, cleared, evidence, now_s):
        record=self.ledger._require(transfer_id)
        if record.stage!='COMMITTED' or not evidence or any(
                cleared.get(name) is not True for name in (record.source,record.receiver)):
            raise SchemaRefused('REFUSED_PRECONDITION_UNKNOWN',
                                'resource release requires measured clearance after commit')
        for name in (record.source,record.receiver):
            grant=self.resources.snapshot(name)
            if grant['owner']!=transfer_id:
                raise SchemaRefused('REFUSED_CONFLICT','reservation owner changed before release')
        for name in (record.source,record.receiver):
            grant=self.resources.snapshot(name)
            self.resources.release(name,owner=transfer_id,generation=grant['generation'],
                                   epoch=self.resources.epoch,now_s=now_s)
        self.ledger.advance(transfer_id,evidence=evidence)
        self._save(record,'released_after_measured_clearance')
